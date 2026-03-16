#include "omniseer/vision/yolo_world_text_embeddings.hpp"

#include <algorithm>
#include <cctype>
#include <codecvt>
#include <cstdint>
#include <fstream>
#include <locale>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rknn_api.h>

namespace omniseer::vision
{
  namespace
  {
    constexpr int kBosTokenId = 49406;
    constexpr int kEosTokenId = 49407;
    constexpr int kPadTokenId = 49407;

    std::runtime_error make_rknn_error(const char* where, int rc)
    {
      return std::runtime_error(std::string(where) + " failed, code=" + std::to_string(rc));
    }

    size_t tensor_bytes(const rknn_tensor_attr& attr) noexcept
    {
      return (attr.size_with_stride != 0) ? attr.size_with_stride : attr.size;
    }

    float clamp_float(float value, float lo, float hi) noexcept
    {
      return std::min(std::max(value, lo), hi);
    }

    int8_t quantize_affine_i8(float value, int32_t zero_point, float scale) noexcept
    {
      const float quantized = (value / scale) + static_cast<float>(zero_point);
      return static_cast<int8_t>(clamp_float(quantized, -128.0F, 127.0F));
    }

    std::string read_text_file(const std::string& path)
    {
      std::ifstream ifs(path, std::ios::binary | std::ios::ate);
      if (!ifs)
        throw std::runtime_error("failed to open file: " + path);

      const std::ifstream::pos_type end = ifs.tellg();
      if (end < 0)
        throw std::runtime_error("failed to stat file: " + path);

      std::string data(static_cast<size_t>(end), '\0');
      ifs.seekg(0, std::ios::beg);
      if (!ifs.read(data.data(), static_cast<std::streamsize>(data.size())))
        throw std::runtime_error("failed to read file: " + path);
      return data;
    }

    std::u32string utf8_to_utf32(const std::string& utf8)
    {
      std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
      return converter.from_bytes(utf8);
    }

    std::string utf32_to_utf8(const std::u32string& utf32)
    {
      std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
      return converter.to_bytes(utf32);
    }

    std::u32string unicode_value_to_utf32(int unicode_value)
    {
      return std::u32string{static_cast<char32_t>(unicode_value)};
    }

    std::vector<std::pair<int, std::u32string>> bytes_to_unicode()
    {
      std::vector<std::pair<int, std::u32string>> byte_unicode_pairs{};
      std::set<int>                               byte_set{};
      for (int b = static_cast<int>('!'); b <= static_cast<int>('~'); ++b)
      {
        byte_set.insert(b);
        byte_unicode_pairs.emplace_back(b, unicode_value_to_utf32(b));
      }
      for (int b = 161; b <= 172; ++b)
      {
        byte_set.insert(b);
        byte_unicode_pairs.emplace_back(b, unicode_value_to_utf32(b));
      }
      for (int b = 174; b <= 255; ++b)
      {
        byte_set.insert(b);
        byte_unicode_pairs.emplace_back(b, unicode_value_to_utf32(b));
      }
      int next_unicode = 0;
      for (int b = 0; b < 256; ++b)
      {
        if (byte_set.find(b) == byte_set.end())
        {
          byte_unicode_pairs.emplace_back(b, unicode_value_to_utf32(next_unicode + 256));
          next_unicode += 1;
        }
      }
      return byte_unicode_pairs;
    }

    std::string strip_ascii_whitespace(const std::string& text)
    {
      const size_t start = text.find_first_not_of(" \t\n\r\v\f");
      if (start == std::string::npos)
        return {};
      const size_t end = text.find_last_not_of(" \t\n\r\v\f");
      return text.substr(start, end - start + 1);
    }

    std::string whitespace_clean(std::string text)
    {
      text = std::regex_replace(text, std::regex(R"(\s+)"), " ");
      return strip_ascii_whitespace(text);
    }

    std::set<std::pair<std::u32string, std::u32string>>
    get_pairs(const std::vector<std::u32string>& subwords)
    {
      std::set<std::pair<std::u32string, std::u32string>> pairs{};
      if (subwords.empty())
        return pairs;

      std::u32string prev = subwords[0];
      for (size_t i = 1; i < subwords.size(); ++i)
      {
        pairs.emplace(prev, subwords[i]);
        prev = subwords[i];
      }
      return pairs;
    }

    class ClipTokenizer
    {
    public:
      explicit ClipTokenizer(const std::string& merges_utf8)
      {
        load_from_merges(merges_utf8);
      }

      std::vector<int> tokenize(std::string text, size_t max_length, bool padding) const
      {
        std::vector<int> tokens = encode(std::move(text));
        tokens.insert(tokens.begin(), kBosTokenId);
        if (max_length > 0)
        {
          if (tokens.size() > max_length - 1)
          {
            tokens.resize(max_length - 1);
            tokens.push_back(kEosTokenId);
          }
          else
          {
            tokens.push_back(kEosTokenId);
            if (padding)
              tokens.insert(tokens.end(), max_length - tokens.size(), kPadTokenId);
          }
        }
        return tokens;
      }

    private:
      void load_from_merges(const std::string& merges_utf8)
      {
        const auto byte_unicode_pairs = bytes_to_unicode();
        _byte_encoder = std::map<int, std::u32string>(byte_unicode_pairs.begin(), byte_unicode_pairs.end());

        std::vector<std::u32string> merges{};
        const std::u32string        merges_utf32 = utf8_to_utf32(merges_utf8);
        size_t                      start        = 0;
        while (start < merges_utf32.size())
        {
          const size_t end = merges_utf32.find(U'\n', start);
          if (end == std::u32string::npos)
          {
            merges.push_back(merges_utf32.substr(start));
            break;
          }
          merges.push_back(merges_utf32.substr(start, end - start));
          start = end + 1;
        }
        if (!merges.empty())
          merges.erase(merges.begin());

        std::vector<std::pair<std::u32string, std::u32string>> merge_pairs{};
        merge_pairs.reserve(merges.size());
        for (const std::u32string& merge : merges)
        {
          const size_t split = merge.find(U' ');
          if (split == std::u32string::npos)
            continue;
          merge_pairs.emplace_back(merge.substr(0, split), merge.substr(split + 1));
        }

        std::vector<std::u32string> vocab{};
        vocab.reserve(byte_unicode_pairs.size() * 2 + merge_pairs.size() + 2);
        for (const auto& pair : byte_unicode_pairs)
          vocab.push_back(pair.second);
        for (const auto& pair : byte_unicode_pairs)
          vocab.push_back(pair.second + utf8_to_utf32("</w>"));
        for (const auto& pair : merge_pairs)
          vocab.push_back(pair.first + pair.second);
        vocab.push_back(utf8_to_utf32("<|startoftext|>"));
        vocab.push_back(utf8_to_utf32("<|endoftext|>"));

        for (size_t i = 0; i < vocab.size(); ++i)
          _encoder[vocab[i]] = static_cast<int>(i);
        for (size_t i = 0; i < merge_pairs.size(); ++i)
          _bpe_ranks[merge_pairs[i]] = static_cast<int>(i);
      }

      std::u32string bpe(const std::u32string& token) const
      {
        std::vector<std::u32string> word{};
        for (size_t i = 0; i + 1 < token.size(); ++i)
          word.emplace_back(1, token[i]);
        word.push_back(token.substr(token.size() - 1) + utf8_to_utf32("</w>"));

        auto pairs = get_pairs(word);
        if (pairs.empty())
          return token + utf8_to_utf32("</w>");

        while (true)
        {
          auto best = pairs.end();
          for (auto it = pairs.begin(); it != pairs.end(); ++it)
          {
            if (_bpe_ranks.find(*it) == _bpe_ranks.end())
              continue;
            if (best == pairs.end() || _bpe_ranks.at(*it) < _bpe_ranks.at(*best))
              best = it;
          }
          if (best == pairs.end())
            break;

          const std::u32string& first  = best->first;
          const std::u32string& second = best->second;
          std::vector<std::u32string> new_word{};
          int32_t                     i = 0;
          while (i < static_cast<int32_t>(word.size()))
          {
            auto it = std::find(word.begin() + i, word.end(), first);
            if (it == word.end())
            {
              new_word.insert(new_word.end(), word.begin() + i, word.end());
              break;
            }
            new_word.insert(new_word.end(), word.begin() + i, it);
            i = static_cast<int32_t>(std::distance(word.begin(), it));

            if (word[i] == first && i < static_cast<int32_t>(word.size()) - 1 && word[i + 1] == second)
            {
              new_word.push_back(first + second);
              i += 2;
            }
            else
            {
              new_word.push_back(word[i]);
              i += 1;
            }
          }

          word = std::move(new_word);
          if (word.size() == 1)
            break;
          pairs = get_pairs(word);
        }

        std::u32string result{};
        for (size_t i = 0; i < word.size(); ++i)
        {
          result += word[i];
          if (i + 1 < word.size())
            result += utf8_to_utf32(" ");
        }
        return result;
      }

      std::vector<int> encode(std::string text) const
      {
        std::vector<int> bpe_tokens{};
        text = whitespace_clean(std::move(text));
        std::transform(text.begin(), text.end(), text.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        static const std::regex pattern(
            R"(<\|startoftext\|>|<\|endoftext\|>|'s|'t|'re|'ve|'m|'ll|'d|[[:alpha:]]+|[[:digit:]]|[^[:space:][:alpha:][:digit:]]+)",
            std::regex::icase);

        std::smatch   matches{};
        std::string   remaining = text;
        while (std::regex_search(remaining, matches, pattern))
        {
          for (const auto& match : matches)
          {
            const std::string token_str = match.str();
            std::u32string    utf32_token{};
            for (unsigned char byte : token_str)
              utf32_token += _byte_encoder.at(byte);

            const std::u32string bpe_tokens_utf32 = bpe(utf32_token);
            size_t               start            = 0;
            while (start < bpe_tokens_utf32.size())
            {
              const size_t end = bpe_tokens_utf32.find(U' ', start);
              const auto   token =
                  (end == std::u32string::npos) ? bpe_tokens_utf32.substr(start)
                                                : bpe_tokens_utf32.substr(start, end - start);
              bpe_tokens.push_back(_encoder.at(token));
              if (end == std::u32string::npos)
                break;
              start = end + 1;
            }
          }
          remaining = matches.suffix();
        }
        return bpe_tokens;
      }

      std::map<int, std::u32string>                                    _byte_encoder{};
      std::map<std::u32string, int>                                    _encoder{};
      std::map<std::pair<std::u32string, std::u32string>, int>         _bpe_ranks{};
    };

    struct RknnModelInfo
    {
      rknn_context                    ctx{0};
      rknn_input_output_num           io_num{};
      std::vector<rknn_tensor_attr>   input_attrs{};
      std::vector<rknn_tensor_attr>   output_attrs{};

      explicit RknnModelInfo(const std::string& model_path)
      {
        if (model_path.empty())
          throw std::invalid_argument("model path is empty");

        const int rc_init = rknn_init(&ctx, const_cast<char*>(model_path.c_str()), 0, 0, nullptr);
        if (rc_init != RKNN_SUCC)
          throw make_rknn_error("rknn_init", rc_init);

        try
        {
          const int rc_io = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
          if (rc_io != RKNN_SUCC)
            throw make_rknn_error("rknn_query(RKNN_QUERY_IN_OUT_NUM)", rc_io);

          input_attrs.resize(io_num.n_input);
          for (uint32_t i = 0; i < io_num.n_input; ++i)
          {
            input_attrs[i]       = rknn_tensor_attr{};
            input_attrs[i].index = i;
            const int rc = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
            if (rc != RKNN_SUCC)
              throw make_rknn_error("rknn_query(RKNN_QUERY_INPUT_ATTR)", rc);
          }

          output_attrs.resize(io_num.n_output);
          for (uint32_t i = 0; i < io_num.n_output; ++i)
          {
            output_attrs[i]       = rknn_tensor_attr{};
            output_attrs[i].index = i;
            const int rc = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
            if (rc != RKNN_SUCC)
              throw make_rknn_error("rknn_query(RKNN_QUERY_OUTPUT_ATTR)", rc);
          }
        }
        catch (...)
        {
          if (ctx != 0)
          {
            (void) rknn_destroy(ctx);
            ctx = 0;
          }
          throw;
        }
      }

      ~RknnModelInfo() noexcept
      {
        if (ctx != 0)
          (void) rknn_destroy(ctx);
      }

      const rknn_tensor_attr& input_by_name(const char* name) const
      {
        for (const auto& attr : input_attrs)
        {
          if (std::string(attr.name) == name)
            return attr;
        }
        throw std::runtime_error(std::string("missing input tensor: ") + name);
      }
    };
  } // namespace

  YoloWorldTextEmbeddingsBuilder::YoloWorldTextEmbeddingsBuilder(
      YoloWorldTextEmbeddingsBuilderConfig cfg)
      : _cfg(std::move(cfg))
  {
  }

  PreparedTextEmbeddings
  YoloWorldTextEmbeddingsBuilder::build(const std::vector<std::string>& class_names) const
  {
    if (class_names.empty())
      throw std::invalid_argument("YoloWorldTextEmbeddingsBuilder::build: class_names is empty");
    if (_cfg.text_encoder_model_path.empty())
      throw std::invalid_argument(
          "YoloWorldTextEmbeddingsBuilder::build: text_encoder_model_path is empty");
    if (_cfg.detector_model_path.empty())
      throw std::invalid_argument(
          "YoloWorldTextEmbeddingsBuilder::build: detector_model_path is empty");
    if (_cfg.clip_vocab_path.empty())
      throw std::invalid_argument("YoloWorldTextEmbeddingsBuilder::build: clip_vocab_path is empty");
    if (_cfg.pad_token.empty())
      throw std::invalid_argument("YoloWorldTextEmbeddingsBuilder::build: pad_token is empty");

    const ClipTokenizer tokenizer(read_text_file(_cfg.clip_vocab_path));
    const RknnModelInfo clip_model(_cfg.text_encoder_model_path);
    const RknnModelInfo detector_model(_cfg.detector_model_path);

    if (clip_model.input_attrs.size() != 1 || clip_model.output_attrs.size() != 1)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: unsupported CLIP text model IO count");

    const rknn_tensor_attr& clip_input  = clip_model.input_attrs[0];
    const rknn_tensor_attr& clip_output = clip_model.output_attrs[0];
    const rknn_tensor_attr& text_input  = detector_model.input_by_name("texts");

    if (clip_input.n_dims < 2)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: unsupported CLIP input tensor rank");
    if (clip_output.n_elems == 0)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: CLIP output tensor has zero elements");
    if (text_input.n_dims < 3 || text_input.dims[0] != 1)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: unsupported detector texts tensor shape");
    if (text_input.type != RKNN_TENSOR_INT8 || text_input.scale <= 0.0F)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: detector texts tensor is not affine INT8");

    const uint32_t sequence_len    = clip_input.dims[clip_input.n_dims - 1];
    const uint32_t class_capacity  = text_input.dims[1];
    const uint32_t embedding_width = text_input.dims[2];
    if (sequence_len == 0 || class_capacity == 0 || embedding_width == 0)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: invalid model tensor dimensions");
    if (class_names.size() > class_capacity)
      throw std::invalid_argument(
          "YoloWorldTextEmbeddingsBuilder::build: class count exceeds detector capacity");
    if (clip_output.n_elems != embedding_width)
      throw std::runtime_error(
          "YoloWorldTextEmbeddingsBuilder::build: CLIP output width does not match detector texts width");

    PreparedTextEmbeddings prepared{};
    prepared.class_names = class_names;
    prepared.text_i8.assign(tensor_bytes(text_input), static_cast<int8_t>(text_input.zp));

    std::vector<std::string> embedding_rows{};
    embedding_rows.reserve(class_capacity);
    embedding_rows.insert(embedding_rows.end(), class_names.begin(), class_names.end());
    embedding_rows.insert(embedding_rows.end(), class_capacity - class_names.size(), _cfg.pad_token);

    std::vector<float> embedding(static_cast<size_t>(embedding_width), 0.0F);
    for (size_t class_index = 0; class_index < embedding_rows.size(); ++class_index)
    {
      const std::vector<int> tokens = tokenizer.tokenize(embedding_rows[class_index], sequence_len, true);
      if (tokens.size() != sequence_len)
        throw std::runtime_error(
            "YoloWorldTextEmbeddingsBuilder::build: tokenizer produced unexpected sequence length");

      rknn_input input{};
      input.index = 0;
      input.fmt   = RKNN_TENSOR_UNDEFINED;
      input.type  = clip_input.type;
      std::vector<int64_t> input_ids64{};
      std::vector<int32_t> input_ids32{};
      if (clip_input.type == RKNN_TENSOR_INT64)
      {
        input_ids64.assign(tokens.begin(), tokens.end());
        input.size = input_ids64.size() * sizeof(int64_t);
        input.buf  = input_ids64.data();

        const int rc_set = rknn_inputs_set(clip_model.ctx, clip_model.io_num.n_input, &input);
        if (rc_set != RKNN_SUCC)
          throw make_rknn_error("rknn_inputs_set", rc_set);
      }
      else if (clip_input.type == RKNN_TENSOR_INT32)
      {
        input_ids32.assign(tokens.begin(), tokens.end());
        input.size = input_ids32.size() * sizeof(int32_t);
        input.buf  = input_ids32.data();

        const int rc_set = rknn_inputs_set(clip_model.ctx, clip_model.io_num.n_input, &input);
        if (rc_set != RKNN_SUCC)
          throw make_rknn_error("rknn_inputs_set", rc_set);
      }
      else
      {
        throw std::runtime_error(
            "YoloWorldTextEmbeddingsBuilder::build: unsupported CLIP input tensor type");
      }

      const int rc_run = rknn_run(clip_model.ctx, nullptr);
      if (rc_run != RKNN_SUCC)
        throw make_rknn_error("rknn_run", rc_run);

      rknn_output output{};
      output.want_float = 1;
      const int rc_get  = rknn_outputs_get(clip_model.ctx, 1, &output, nullptr);
      if (rc_get != RKNN_SUCC)
        throw make_rknn_error("rknn_outputs_get", rc_get);

      try
      {
        if (output.buf == nullptr)
          throw std::runtime_error(
              "YoloWorldTextEmbeddingsBuilder::build: CLIP output buffer is null");
        const float* values = static_cast<const float*>(output.buf);
        std::copy(values, values + embedding.size(), embedding.begin());
      }
      catch (...)
      {
        (void) rknn_outputs_release(clip_model.ctx, 1, &output);
        throw;
      }

      const int rc_release = rknn_outputs_release(clip_model.ctx, 1, &output);
      if (rc_release != RKNN_SUCC)
        throw make_rknn_error("rknn_outputs_release", rc_release);

      const size_t base = class_index * embedding_width;
      for (size_t i = 0; i < embedding.size(); ++i)
      {
        prepared.text_i8[base + i] = quantize_affine_i8(embedding[i], text_input.zp, text_input.scale);
      }
    }

    return prepared;
  }
} // namespace omniseer::vision

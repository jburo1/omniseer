#pragma once

#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

namespace robot_diag_control_cpp
{
class GrpcServer
{
public:
  explicit GrpcServer(
    grpc::Service & service,
    std::string bind_address = "0.0.0.0",
    int port = 50051);
  ~GrpcServer();

  void start();
  void stop();

  [[nodiscard]] int port() const;
  [[nodiscard]] std::string listen_address() const;

private:
  grpc::Service & _service;
  std::string _bind_address;
  int _requested_port{50051};
  int _bound_port{0};
  std::unique_ptr<grpc::Server> _server{};
};
} // namespace robot_diag_control_cpp

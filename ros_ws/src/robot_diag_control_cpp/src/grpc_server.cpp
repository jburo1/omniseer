#include "robot_diag_control_cpp/grpc_server.hpp"

#include <stdexcept>
#include <utility>

namespace robot_diag_control_cpp
{
GrpcServer::GrpcServer(grpc::Service & service, std::string bind_address, int port)
: _service(service),
  _bind_address(std::move(bind_address)),
  _requested_port(port)
{
}

GrpcServer::~GrpcServer()
{
  stop();
}

void GrpcServer::start()
{
  if (_server) {
    return;
  }

  grpc::ServerBuilder builder;
  const std::string endpoint = _bind_address + ":" + std::to_string(_requested_port);
  int selected_port = 0;
  builder.AddListeningPort(endpoint, grpc::InsecureServerCredentials(), &selected_port);
  builder.RegisterService(&_service);

  _server = builder.BuildAndStart();
  if (!_server || selected_port == 0) {
    _server.reset();
    throw std::runtime_error("failed to bind gRPC server port");
  }

  _bound_port = selected_port;
}

void GrpcServer::stop()
{
  if (!_server) {
    return;
  }

  _server->Shutdown();
  _server->Wait();
  _server.reset();
  _bound_port = 0;
}

int GrpcServer::port() const
{
  return _bound_port;
}

std::string GrpcServer::listen_address() const
{
  return _bind_address + ":" + std::to_string(_bound_port);
}
} // namespace robot_diag_control_cpp

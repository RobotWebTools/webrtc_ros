#ifndef CPP_WEB_SERVER_HTTP_HEADER_HPP
#define CPP_WEB_SERVER_HTTP_HEADER_HPP

#include <string>

namespace cpp_web_server
{

struct HttpHeader
{
  HttpHeader()
  {
  }

  HttpHeader(std::string name, std::string value) : name(name), value(value)
  {
  }

  std::string name;
  std::string value;
};

}

#endif

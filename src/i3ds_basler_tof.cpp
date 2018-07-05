///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <csignal>
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <memory>

#include <boost/program_options.hpp>

#include "basler_tof_camera.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace po = boost::program_options;
namespace logging = boost::log;


volatile bool running;

void signal_handler(int signum)
{
  BOOST_LOG_TRIVIAL(info) << "do_deactivate()";
  running = false;
}



int main(int argc, char** argv)
{
  unsigned int node_id;
  i3ds::BaslerToFCamera::Parameters param;

  po::options_description desc("Allowed camera control options");
  desc.add_options()
  ("help,h", "Produce this message")
  ("node,n", po::value<unsigned int>(&node_id)->default_value(12), "Node ID of camera")
  ("camera-name,c", po::value<std::string>(&param.camera_name)->default_value("i3ds-basler-tof"), "Connect via (UserDefinedName) of Camera")
  ("free-running,f", po::bool_switch(&param.free_running)->default_value(false), "Free-running sampling. Default external triggered")

  ("verbose,v", "Print verbose output")
  ("quite,q", "Quiet ouput")
  ("print,p", "Print the camera configuration")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return -1;
    }

  if (vm.count("quite"))
    {
      logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::warning);
    }
  else if (!vm.count("verbose"))
    {
      logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::info);
    }

  po::notify(vm);

  BOOST_LOG_TRIVIAL(info) << "Using Nodeid: " << node_id;

  i3ds::Context::Ptr context = i3ds::Context::Create();;

  i3ds::Server server(context);

  i3ds::BaslerToFCamera camera(context, node_id, param);

  camera.Attach(server);

  running = true;
  signal(SIGINT, signal_handler);

  server.Start();

  while (running)
    {
      sleep(1);
    }

  server.Stop();

  return 0;
}

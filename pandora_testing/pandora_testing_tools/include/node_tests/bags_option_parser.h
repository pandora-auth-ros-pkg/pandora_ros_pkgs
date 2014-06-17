#ifndef BAGS_OPTION_PARSER_H
#define BAGS_OPTION_PARSER_H

#include "rosbag/player.h"
#include "boost/program_options.hpp"

namespace po = boost::program_options;

rosbag::PlayerOptions parseOptions(int argc, char** argv) {
	rosbag::PlayerOptions opts;

	po::options_description desc("Allowed options");

	desc.add_options()
	  ("help,h", "produce help message")
	  ("quiet,q", "suppress console output")
	  ("immediate,i", "play back all messages without waiting")
	  ("pause", "start in paused mode")
	  ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
	  ("clock", "publish the clock time")
	  ("hz", po::value<float>()->default_value(100.0), "use a frequency of HZ when publishing clock time")
	  ("delay,d", po::value<float>()->default_value(0.2), "sleep SEC seconds after every advertise call")
	  ("rate,r", po::value<float>()->default_value(1.0), "multiply the publish rate by FACTOR")
	  ("start,s", po::value<float>()->default_value(0.0), "start SEC seconds into the bag files")
	  ("loop,l", "loop playback")
	  ("keep-alive,k", "keep alive past end of bag")
	  ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
	  ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
	  ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
	  ("bags", po::value< std::vector<std::string> >(), "bag files to play back from");
	
	po::positional_options_description p;
	p.add("bags", -1);
	
	po::variables_map vm;
	
	try 
	{
	  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	} catch (boost::program_options::invalid_command_line_syntax& e)
	{
	  throw ros::Exception(e.what());
	}  catch (boost::program_options::unknown_option& e)
	{
	  throw ros::Exception(e.what());
	}

	if (vm.count("help")) {
	  std::cout << desc << std::endl;
	  exit(0);
	}

	if (vm.count("quiet"))
	  opts.quiet = true;
	if (vm.count("immediate"))
	  opts.at_once = true;
	if (vm.count("pause"))
	  opts.start_paused = true;
	if (vm.count("queue"))
	  opts.queue_size = vm["queue"].as<int>();
	if (vm.count("hz"))
	  opts.bag_time_frequency = vm["hz"].as<float>();
	if (vm.count("clock"))
	  opts.bag_time = true;
	if (vm.count("delay"))
	  opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
	if (vm.count("rate"))
	  opts.time_scale = vm["rate"].as<float>();
	if (vm.count("start"))
	{
	  opts.time = vm["start"].as<float>();
	  opts.has_time = true;
	}
	if (vm.count("skip-empty"))
	  opts.skip_empty = ros::Duration(vm["skip-empty"].as<float>());
	if (vm.count("loop"))
	  opts.loop = true;
	if (vm.count("keep-alive"))
	  opts.keep_alive = true;

	if (vm.count("topics"))
	{
	  std::vector<std::string> topics = vm["topics"].as< std::vector<std::string> >();
	  for (std::vector<std::string>::iterator i = topics.begin();
		   i != topics.end();
		   i++)
		opts.topics.push_back(*i);
	}

	if (vm.count("bags"))
	{
	  std::vector<std::string> bags = vm["bags"].as< std::vector<std::string> >();
	  for (std::vector<std::string>::iterator i = bags.begin();
		   i != bags.end();
		   i++)
		  opts.bags.push_back(*i);
	} else {
	  if (vm.count("topics"))
		throw ros::Exception("When using --topics, --bags should be specified to list bags.");
	  throw ros::Exception("You must specify at least one bag to play back.");
	}
			
	return opts;
}


#endif



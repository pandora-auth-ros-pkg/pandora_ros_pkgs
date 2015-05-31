#ifndef CLOCKPLUSPLUC_HPP
#define CLOCKPLUSPLUC_HPP

#include <iostream>
#include <cstdlib>
#include <string>
#include <map>
#include <set>
#include <fstream>
#include <sys/time.h>

#include "defines.h"

class Timer
{
  typedef std::map<std::string,double> Msd;
  typedef std::map<float,std::string> Mfs;
  typedef std::map<float,std::string>::iterator MfsIt;
  typedef std::map<std::string,unsigned long> Msul;
  typedef std::map<std::string,double>::iterator MsdIt;
  typedef std::map<std::string,double>::const_iterator MsdCIt;
  typedef std::pair<std::string,double> Psd;
  typedef std::pair<std::string,unsigned long> Psul;

  static Msd times;
  static Msd max_time;
  static Msd min_time;
  static Msd mean_time;
  static Msd sum_time;
  static Msul count;
  static struct timeval msTime;
  static std::map<std::string,std::set<std::string> > timer_tree;

  static std::string top_node;

  Timer(void){}

  static void printMsInternal(double t);
  static void printSecInternal(double t);
  static void printMinInternal(double t);
  static void printHoursInternal(double t);
  static void printLiteralInternal(double t);

  public:

  static void start(std::string timerId, std::string father = "", bool top = false);
  static double stop(std::string timerId);
  static double mean(std::string timerId);
  static void tick(std::string timerId);
  static void printMs(std::string timerId);
  static void printSec(std::string timerId);
  static void printMin(std::string timerId);
  static void printHours(std::string timerId);
  static void printLiteral(std::string timerId);
  static void printLiteralMean(std::string timerId, std::string identation = "");
  static void printAll(void);
  static void printAllMeans(void);
  static void printAllMeansTree(void);
  static void printIterativeTree(std::string node, std::string identation);
};

#endif

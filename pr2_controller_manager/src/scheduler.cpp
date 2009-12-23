/////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/*
 * Author: Wim Meeussen
 */

#include "pr2_controller_manager/scheduler.h"

using namespace std;

typedef map<string, list<string> > schedGraph;




bool getNextController(string& c, schedGraph& graph)
{
  schedGraph::iterator it, it2;
  for (it = graph.begin(); it != graph.end(); it++){
    // found next controller to schedule
    if (it->second.empty()){
      c = it->first;
      // remove this controller form graph 
      graph.erase(it);
      // remove this controller form before lists
      for (it2 = graph.begin(); it2 != graph.end(); it2++){
        list<string>::iterator l=it2->second.begin();
        while(l!=it2->second.end()){
          if ((*l) == c){
            l = it2->second.erase(l);
          }
          else l++;
        }
      }
      return true;
    }
  }
  return false;
}



bool scheduleControllers(const vector<ControllerSpec>& c, vector<size_t>& schedule)
{
  schedGraph graph;
  schedGraph::iterator it;

  schedule.resize(c.size());

  // build graph
  for (size_t i=0; i<c.size(); i++){
    graph[c[i].name];
    for (size_t b=0; b<c[i].c->before_list_.size(); b++)
      graph[c[i].name].push_back(c[i].c->before_list_[b]);
    for (size_t a=0; a<c[i].c->after_list_.size(); a++){
      it = graph.find(c[i].c->after_list_[a]);
      if (it == graph.end()) return false;
      it->second.push_back(c[i].name);
    }
  }

  // do the scheduling
  string name;
  size_t nr=0;
  while (!graph.empty()){
    if (!getNextController(name, graph)) return false;

    // find controller id
    for (size_t i=0; i<c.size(); i++)
      if (c[i].name == name)
        schedule[nr] = i;
    nr++;
  }

  // show result
  string schedule_list;
  for (size_t i=0; i<schedule.size(); i++)
    schedule_list += c[schedule[i]].name + ",  ";
  ROS_DEBUG("Controller schedule: %s", schedule_list.c_str());

  return true;
}


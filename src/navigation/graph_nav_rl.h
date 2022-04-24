//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    graph_nav_rl.h
\brief   Slim wrapper to simulator ang graph nav for reinforcement learning.
\author  Joydeep Biswas, (C) 2022
*/
//========================================================================

#ifndef GRAPH_NAV_RL_H
#define GRAPH_NAV_RL_H

namespace graph_nav_rl {

extern "C" int Test(const char* str);

extern "C" void Init();

extern "C" void Step();

extern "C" void Reset();

}  // namespace graph_nav_rl
#endif  // GRAPH_NAV_RL_H
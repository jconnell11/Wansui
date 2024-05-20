// jhcAzureReco.h : web interface to Microsoft Azure speech recognizer
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

#pragma once


//= Web interface to Microsoft Azure speech recognizer.
// Note: can only be one copy since relies on unique global variables

class jhcAzureReco
{
// PUBLIC MEMBER FUNCTIONS
public:
  // main functions
  ~jhcAzureReco ();
  jhcAzureReco ();
  int Start (const char *path =NULL, int prog =0);

  // main functions
  int Status ();
  const char *Heard (char *txt);
  void Done ();

};

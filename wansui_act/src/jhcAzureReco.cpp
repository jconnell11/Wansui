// jhcAzureReco.cpp : web interface to Microsoft Azure speech recognizer
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

#include <unistd.h>

#include <jhcAzureReco.h>

#include <speechapi_cxx.h>

using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;


///////////////////////////////////////////////////////////////////////////

//= Core Azure speech recognition engine.

static std::shared_ptr<SpeechRecognizer> sp = NULL;


//= Full speech recognition result string.

static char full[200] = "";


//= Current speech recognition status.

static int rc = -2;


//= Whether to show partial recognition results for debugging.

static int show = 0;


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcAzureReco::~jhcAzureReco ()
{
  Done();
}


//= Default constructor initializes certain values.

jhcAzureReco::jhcAzureReco ()
{
}


//= Connect to speech recognition engine using stored credentials.
// bind member variable "reco" if successful
// returns 1 if successful, 0 or negative for problem

int jhcAzureReco::Start (const char *path, int prog)
{
  char line[200], key[80], reg[20];
  FILE *in;
  char *end;
  int i;

  // get rid of any previous connection and init vars
  show = prog;
  Done();

  // read license key and geographical area from file
  sprintf(line, "%s/config/ms_azure.key", path);
  if ((in = fopen(line, "r")) == NULL)
    return -2;
  if (fgets(line, 200, in) == NULL)
    return -1;
  if (sscanf(line, "%s", key) != 1)
    return -1;
  if (fgets(line, 200, in) == NULL)
    return -1;
  if (sscanf(line, "%s", reg) != 1)
    return -1;
  fclose(in);

  // build speech recognizer and connect to microphone
  auto cfg = SpeechConfig::FromSubscription(key, reg);
  cfg->SetSpeechRecognitionLanguage("en-US");
  auto mic = AudioConfig::FromDefaultMicrophoneInput();
  sp = SpeechRecognizer::FromConfig(cfg, mic);
 
  // hook up actual recognition events to callbacks
  sp->Recognizing.Connect([](const SpeechRecognitionEventArgs& e)
    {
      if (show > 0)
        printf("  %s ...\n", e.Result->Text.c_str());
      rc = 1;
    });
  sp->Recognized.Connect([](const SpeechRecognitionEventArgs& e)
    {
      if (e.Result->Reason != ResultReason::RecognizedSpeech)
        rc = -1;
      else
      {
        strcpy(full, e.Result->Text.c_str());
        rc = 2;
      }
    });

  // hook up other events to callbacks
  sp->SessionStarted.Connect([](const SessionEventArgs& e)
    {*full = '\0'; rc = 0;});
  sp->Canceled.Connect([](const SpeechRecognitionCanceledEventArgs& e)
    {*full = '\0'; rc = -1;});
  sp->SessionStopped.Connect([](const SessionEventArgs& e)
    {*full = '\0'; rc = -2;});

  // start processing speech and wait for ready status
  sp->StartContinuousRecognitionAsync().get();
  for (i = 0; i < 20; i++)
  {
    if (Status() >= 0)
      return 1;
    usleep(50000);
  }
  return 0;
}


///////////////////////////////////////////////////////////////////////////
//                             Main Functions                            //
///////////////////////////////////////////////////////////////////////////

//= Check to see if a new utterance is available.
// return: 2 new result, 1 speaking, 0 silence, negative for error

int jhcAzureReco::Status ()
{
  int val = rc;
    
  if (val > 0)
    rc = 0;
  return val;
}


//= Get last complete utterance heard by the speech recognizer.
  
const char *jhcAzureReco::Heard (char *txt)
{
  return full;
}


//= Cleanly disconnect and exit speech recognition.

void jhcAzureReco::Done ()
{
  if (sp != NULL)
    sp->StopContinuousRecognitionAsync().get();
  sp = NULL;
  *full = '\0';
  rc = -2;
}



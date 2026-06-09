// jhcAzureReco.cpp : web interface to Microsoft Azure speech recognizer
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024-2025 Etaoin Systems
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


//= Special phrase list for adding people's names.

static std::shared_ptr<PhraseListGrammar> vocab = NULL;


//= Whether to show partial recognition results for debugging.

static int show = 0;


//= Current speech recognition status (used by callbacks).

static int reco = -2;


//= Result duration (ms) plus endpoint silence (used by callbacks);

static int delay = 0;


//= Run-on utterance chunking variables (used by callbacks).

static char blob[500] = "";
static const char *rd = blob;


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
  char line[200], name[80], key[80], reg[20];
  FILE *in;
  char *end;
  int i, n;

  // get rid of any previous connection and init vars
  show = prog;
  Done();

  // read license key and geographical area from file
  sprintf(line, "%sconfig/ms_azure.key", ((path == NULL) ? "" : path));
  if ((in = fopen(line, "r")) == NULL)
    return -3;
  if (fgets(line, 200, in) == NULL)
    return -2;
  if (sscanf(line, "%s", key) != 1)
    return -2;
  if (fgets(line, 200, in) == NULL)
    return -2;
  if (sscanf(line, "%s", reg) != 1)
    return -2;
  fclose(in);

  // check for reasonable format
  if (((int) strlen(key) != 32) || (*reg == '\0'))             
    return -1;                                        
  for (i = 0; i < 32; i++)
    if (isxdigit(key[i]) == 0)
      return -1;    

  // build speech recognizer and connect to microphone
  auto cfg = SpeechConfig::FromSubscription(key, reg);
  cfg->SetSpeechRecognitionLanguage("en-US");
  cfg->SetProfanity(ProfanityOption::Raw);
  cfg->SetProperty(PropertyId::Speech_SegmentationSilenceTimeoutMs, "500");
  auto mic = AudioConfig::FromDefaultMicrophoneInput();
  sp = SpeechRecognizer::FromConfig(cfg, mic);

  // add proper spellings of names from file: config/all_names.txt
  vocab = PhraseListGrammar::FromRecognizer(sp);       
  sprintf(line, "%s/config/all_names.txt", ((path == NULL) ? "." : path));
  if ((in = fopen(line, "r")) != NULL)
  {
    while (fgets(name, 80, in) != NULL)
      if ((n = (int) strlen(name)) > 0)
      {
        name[n - 1] = '\0';
        vocab->AddPhrase(name);                        // limit of 1024
      }
    fclose(in);
  }                             

  // hook up actual recognition events to callbacks
  sp->Recognizing.Connect([](const SpeechRecognitionEventArgs& e)
    {
      if (show > 0)
        printf("  %s ...\n", e.Result->Text.c_str());
      reco = 1;
    });
  sp->Recognized.Connect([](const SpeechRecognitionEventArgs& e)
    {
      if (e.Result->Reason == ResultReason::RecognizedSpeech)
      {
        const char *res = (e.Result->Text).data(); 
  
        if ((*res != '\0') && (strcmp(res, "Hey, Cortana.") != 0))   // quirk
        {
          delay = (int)(0.0001 * e.Result->Duration());
          strcpy(blob, e.Result->Text.c_str());
          rd = blob;
          reco = 2;
        }
      }
      else if (e.Result->Reason == ResultReason::NoMatch)
        reco = -1;
    });

  // hook up other events to callbacks
  sp->SessionStarted.Connect([](const SessionEventArgs& e)
    {reco = 0;});
  sp->Canceled.Connect([](const SpeechRecognitionCanceledEventArgs& e)
    {reco = -1;});
  sp->SessionStopped.Connect([](const SessionEventArgs& e)
    {reco = -2;});

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

//= Add a particular name to grammar to increase likelihood of correct spelling.
// can be called even when recognition is actively running

void jhcAzureReco::Name (const char *person)
{
  vocab->AddPhrase(person);                      // limit of 1024
}


//= Check to see if a new utterance is available.
// return: 2 new result, 1 speaking, 0 silence, negative for error

int jhcAzureReco::Status ()
{
  return reco;
}


//= Gives text string of last full recognition result (changes status).
  
const char *jhcAzureReco::Heard (char *txt)
{
  next_chunk();
  if (txt != NULL)
    strcpy(txt, heard);
  return txt;
}


//= Gives approximate time (ms) that utterance started before notification.

int jhcAzureReco::Delay ()
{
  return delay;
}


//= Cleanly disconnect and exit speech recognition.

void jhcAzureReco::Done ()
{
  if (sp != NULL)
    sp->StopContinuousRecognitionAsync().get();
  sp = NULL;
  reco = -2;
}


///////////////////////////////////////////////////////////////////////////
//                           Response Chunking                           //
///////////////////////////////////////////////////////////////////////////

//= Use Inverse-Text-Normalized form to help break up long lexical results.
// blob = "I saw you in London. I then went to France."
// call 1 --> heard = "I saw you in London."   + reco = 2 (still)
// call 2 --> heard = "I then went to France." + reco = 0 (done)

void jhcAzureReco::next_chunk () 
{
  char abbr[6][10] = {"Dr", "Mr", "Ms", "Mrs", "Prof", "St"};
  const char *end, *scan = rd;
  int i, n, bk;

  // clear output then sanity check
  *heard = '\0';
  if (reco < 2)
    return;

  // look for terminal punctuation
  while ((end = strpbrk(scan, ".!?")) != NULL)
  {
    // ? and ! end immediately but ignore . inside word (e.g. "3.14")
    if (*end != '.')
      break;
    if (end[1] != ' ')  
    {
      scan = end + 1;
      continue;
    }
    n = (int)(end - rd);

    // possibly expand "Dr" to "drive" (not if "Dr. Jones")
    if ((n >= 2) && (strncmp(end - 2, "Dr", 2) == 0))   
      if ((end[1] != ' ') || !isupper(end[2]) || (strchr("FB", end[2]) != NULL))       
      {
        // copy string without "Dr" then add replacement
        strncat(heard, rd, n - 2);  
        strcat(heard, "drive");
        if (end[1] == '\0')             // end of blob
          strcat(heard, ".");         

        // keep scanning after any trailing space
        rd = end + 1;
        scan = rd;
        continue;
      }

    // check for allowable abbreviations
    for (i = 0; i < 6; i++)
      if ((bk = (int) strlen(abbr[i])) <= n)
        if (strncmp(end - bk, abbr[i], bk) == 0)
          break;
    if (i >= 6)                         // true end found
      break;
    scan = end + 1; 
  }

  // copy rest of allowed portion of string
  if (end != NULL)
    n = (int)(end - rd) + 1;
  else
    n = (int) strlen(rd);
  strncat(heard, rd, n);

  // set up for remainder of blob (if any)
  rd += n;
  if (rd[0] != '\0')
    rd++;                      // skip space
  else
    reco = 0;                  
}




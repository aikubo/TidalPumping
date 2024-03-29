//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html
#include "TidalPumpingTestApp.h"
#include "TidalPumpingApp.h"
#include "Moose.h"
#include "AppFactory.h"
#include "MooseSyntax.h"

InputParameters
TidalPumpingTestApp::validParams()
{
  InputParameters params = TidalPumpingApp::validParams();
  params.set<bool>("use_legacy_material_output") = false;
  return params;
}

TidalPumpingTestApp::TidalPumpingTestApp(InputParameters parameters) : MooseApp(parameters)
{
  TidalPumpingTestApp::registerAll(
      _factory, _action_factory, _syntax, getParam<bool>("allow_test_objects"));
}

TidalPumpingTestApp::~TidalPumpingTestApp() {}

void
TidalPumpingTestApp::registerAll(Factory & f, ActionFactory & af, Syntax & s, bool use_test_objs)
{
  TidalPumpingApp::registerAll(f, af, s);
  if (use_test_objs)
  {
    Registry::registerObjectsTo(f, {"TidalPumpingTestApp"});
    Registry::registerActionsTo(af, {"TidalPumpingTestApp"});
  }
}

void
TidalPumpingTestApp::registerApps()
{
  registerApp(TidalPumpingApp);
  registerApp(TidalPumpingTestApp);
}

/***************************************************************************************************
 *********************** Dynamic Library Entry Points - DO NOT MODIFY ******************************
 **************************************************************************************************/
// External entry point for dynamic application loading
extern "C" void
TidalPumpingTestApp__registerAll(Factory & f, ActionFactory & af, Syntax & s)
{
  TidalPumpingTestApp::registerAll(f, af, s);
}
extern "C" void
TidalPumpingTestApp__registerApps()
{
  TidalPumpingTestApp::registerApps();
}

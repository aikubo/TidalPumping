#include "TidalPumpingApp.h"
#include "Moose.h"
#include "AppFactory.h"
#include "ModulesApp.h"
#include "MooseSyntax.h"

InputParameters
TidalPumpingApp::validParams()
{
  InputParameters params = MooseApp::validParams();
  params.set<bool>("use_legacy_material_output") = false;
  return params;
}

TidalPumpingApp::TidalPumpingApp(InputParameters parameters) : MooseApp(parameters)
{
  TidalPumpingApp::registerAll(_factory, _action_factory, _syntax);
}

TidalPumpingApp::~TidalPumpingApp() {}

void 
TidalPumpingApp::registerAll(Factory & f, ActionFactory & af, Syntax & s)
{
  ModulesApp::registerAllObjects<TidalPumpingApp>(f, af, s);
  Registry::registerObjectsTo(f, {"TidalPumpingApp"});
  Registry::registerActionsTo(af, {"TidalPumpingApp"});

  /* register custom execute flags, action syntax, etc. here */
}

void
TidalPumpingApp::registerApps()
{
  registerApp(TidalPumpingApp);
}

/***************************************************************************************************
 *********************** Dynamic Library Entry Points - DO NOT MODIFY ******************************
 **************************************************************************************************/
extern "C" void
TidalPumpingApp__registerAll(Factory & f, ActionFactory & af, Syntax & s)
{
  TidalPumpingApp::registerAll(f, af, s);
}
extern "C" void
TidalPumpingApp__registerApps()
{
  TidalPumpingApp::registerApps();
}

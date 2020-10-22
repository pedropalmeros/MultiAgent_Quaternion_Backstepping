// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   xyFleetBkstp_impl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/
#include "xyFleetBkstp_impl.h"
#include "xyFleetBkstp.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <vector>

using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

xyFleetBkstp_impl::xyFleetBkstp_impl(xyFleetBkstp *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 9, 1, floatType, name);

  MatrixDescriptor *desc = new MatrixDescriptor(5, 1);
  desc->SetElementName(0, 0, "x_0");
  desc->SetElementName(1, 0, "x_1");
  desc->SetElementName(2, 0, "x_2");
  desc->SetElementName(3, 0, "Ctrl");
  desc->SetElementName(4, 0, "Protocol");
  state = new Matrix(self, desc, floatType, name);
  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",0, 1, 0.01);
  //alpha_1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "Kp:", 0, 90000000, 0.01);
  //ki = new DoubleSpinBox(reglages_groupbox->NewRow(), "ki:", 0, 90000000, 0.01,3);
  //sati = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat i:", 0, 1,0.01);
  //kd = new DoubleSpinBox(reglages_groupbox->NewRow(), "kd:", -10, 90000000, 0.001);
  //kp = new DoubleSpinBox(reglages_groupbox->NewRow(), "FG_01: ", -10, 10, 0.001);

  //FGain_01 = new DoubleSpinBox(reglages_groupbox->NewRow(), "FG_01: ", -10, 10, 0.001);
  sat_consensus = new DoubleSpinBox(reglages_groupbox->NewRow(), "Sat_Consensus: ", -10, 10, 0.001);
  alpha_1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_1 (Bkstp): ", -10, 10, 0.001);
  alpha_2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_2 (Bsktp): ", -10, 10, 0.001);
  //f1      = new DoubleSpinBox(reglages_groupbox->NewRow(), "factor  (Bkstp): ", -10, 10, 0.001);

  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", 0, 1, 0.1);
  //sat_ec = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_ec:", -10, 10, 0.1);
}

xyFleetBkstp_impl::~xyFleetBkstp_impl(void) {
}

void xyFleetBkstp_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);
  plot->AddCurve(state->Element(4), DataPlot::Red);

}

void xyFleetBkstp_impl::Data4Formation(const LayoutPosition *position){

}

void xyFleetBkstp_impl::UpdateFrom(const io_data *data) {

  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  input->GetMutex();

  std::vector<float> States{0,0,0};
  std::vector<float> RelDist{0,0,0};

  int id = (int)(input->ValueNoMutex(0,0));

  States[0] = input->ValueNoMutex(1,0);
  States[1] = input->ValueNoMutex(2,0);
  States[2] = input->ValueNoMutex(3,0);
  RelDist[0] = input->ValueNoMutex(4,0);
  RelDist[1] = input->ValueNoMutex(5,0);
  RelDist[2] = input->ValueNoMutex(6,0);

  
  float vel = input->ValueNoMutex(7,0);
  float ref_vel = input->ValueNoMutex(8,0);


  input->ReleaseMutex();

  // Here e1 has been already saturated. 
  float e1 = FormationProtocol(id,States,RelDist);

  float ctrl2virtual = -alpha_1->Value()*e1 + ref_vel;

  float e2 = vel - ctrl2virtual;

  float u = - alpha_1->Value()*(e2 - alpha_1->Value()*e1) - alpha_2->Value()*e2  - e1;

  float u_total = u;






  //cout << "u_tot: " << u_total;
  OwnSat(u_total,sat_u->Value() );
  //cout << "                 u_tot_sat: " << u_total << endl; 
  

  state->GetMutex();
  state->SetValueNoMutex(0, 0, States[0]);
  state->SetValueNoMutex(1, 0, States[1]);
  state->SetValueNoMutex(2, 0, States[2]);
  state->SetValueNoMutex(3, 0, e1);
  state->SetValueNoMutex(4, 0, u_total);
  state->ReleaseMutex();

  //cout << "RelaseMutex ok " << endl; 

  self->output->SetValue(0, 0, u_total);
  self->output->SetDataTime(data->DataTime());
}


void xyFleetBkstp_impl::OwnSat(float& signal, double saturation_value){
  if (signal > saturation_value){
    signal = saturation_value;
  }
  else if(signal < -saturation_value){
    signal = -saturation_value;
  }
  else
    signal =  signal;
}

float xyFleetBkstp_impl::SatVal(float value, double saturation_value){
  if (value > saturation_value){
    return saturation_value;
  }
  else if(value < -saturation_value){
    return -saturation_value;
  }
  else
    return  value;
}


float xyFleetBkstp_impl::FormationProtocol(int id, std::vector<float> &states, std::vector<float> &reldist){
  //cout<< "inside FormationProtocl" << endl; 
  float u = 0.0;
  for(int i = 0; i < 3; i++){
    u += states[id] - states[i]+reldist[i];
  }

  // This is done to saturate the consensus protocol. 
  OwnSat(u,sat_consensus->Value());

  return u;
}
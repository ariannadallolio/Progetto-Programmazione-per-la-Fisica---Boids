#include "histogram.hpp"

#include <fstream>
#include <iostream>

#include "TCanvas.h"
#include "TH1F.h"

namespace pf {
void graph() {
  TH1F *h_dist = new TH1F(
      "h_dist",
      "History of Boids' Mean Distance;Distance;Frequency (numbers of frame)",
      50, 0.0, 700.0);
  TH1F *h_vel = new TH1F(
      "h_vel",
      "History of Flock's Mean Velocity;Velocity;Frequency (numbers of frame)",
      50, 0.0, 50.0);

  std::ifstream file("statistics.txt");
  if (!file.is_open()) {
    throw std::runtime_error{"Impossible to read statistics.txt"};
  }

  double time, mean_d, std_d, mean_v, std_v;
  while (file >> time >> mean_d >> std_d >> mean_v >> std_v) {
    h_dist->Fill(mean_d);
    h_vel->Fill(mean_v);
  }

  TCanvas *c1 = new TCanvas("c1", "Flock's Statistics", 1200, 500);
  c1->Divide(2, 1);

  c1->cd(1);
  h_dist->SetFillColor(kRed);
  h_dist->Draw();

  c1->cd(2);
  h_vel->SetFillColor(kBlue);
  h_vel->Draw();

  c1->SaveAs("flock_statistics.png");
  file.close();
}
}  // namespace pf

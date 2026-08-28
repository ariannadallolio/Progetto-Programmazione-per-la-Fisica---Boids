#include "graphs.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "TCanvas.h"
#include "TError.h"
#include "TGraphErrors.h"
#include "TH1F.h"
#include "TROOT.h"

namespace pf {

void draw_graphs() {
  gROOT->SetBatch(kTRUE);
  gErrorIgnoreLevel = kWarning;  // to ignore messages on the shell

  std::ifstream file{"statistics.txt"};
  if (!file.is_open()) {
    throw std::runtime_error{"Error: cannot read statistics.txt"};
  }

  std::vector<double> time;
  std::vector<double> mean_dist;
  std::vector<double> std_dist;
  std::vector<double> mean_vel;
  std::vector<double> std_vel;

  double t, md, sd, mv, sv;
  while (file >> t >> md >> sd >> mv >> sv) {
    time.push_back(t);
    mean_dist.push_back(md);
    std_dist.push_back(sd);
    mean_vel.push_back(mv);
    std_vel.push_back(sv);
  }

  int const n = static_cast<int>(time.size());
  if (n == 0) {
    throw std::runtime_error{"Error: no data found in file"};
  }

  // to center histograms
  auto dist_bounds = std::minmax_element(mean_dist.begin(), mean_dist.end());
  auto vel_bounds = std::minmax_element(mean_vel.begin(), mean_vel.end());

  double const dist_margin =
      0.05 * (*dist_bounds.second - *dist_bounds.first) + 1e-6;
  double const vel_margin =
      0.05 * (*vel_bounds.second - *vel_bounds.first) + 1e-6;

  TH1F h_dist{"h_dist", "Distribution of mean distance;distance;frames", 50,
              *dist_bounds.first - dist_margin,
              *dist_bounds.second + dist_margin};
  TH1F h_vel{"h_vel", "Distribution of mean speed;speed;frames", 50,
             *vel_bounds.first - vel_margin, *vel_bounds.second + vel_margin};

  for (int i = 0; i < n; ++i) {
    h_dist.Fill(mean_dist[static_cast<std::size_t>(i)]);
    h_vel.Fill(mean_vel[static_cast<std::size_t>(i)]);
  }

  h_dist.SetFillColor(kRed);
  h_vel.SetFillColor(kBlue);

  TGraphErrors g_dist{n, time.data(), mean_dist.data(), nullptr,
                      std_dist.data()};
  g_dist.SetTitle("Mean distance over time;time [s];distance");
  g_dist.SetFillColor(kRed - 9);
  g_dist.SetLineColor(kRed);
  g_dist.SetLineWidth(2);

  TGraphErrors g_vel{n, time.data(), mean_vel.data(), nullptr, std_vel.data()};
  g_vel.SetTitle("Mean speed over time;time [s];speed");
  g_vel.SetFillColor(kBlue - 9);
  g_vel.SetLineColor(kBlue);
  g_vel.SetLineWidth(2);

  TCanvas canvas{"canvas", "Flock statistics", 1200, 900};
  canvas.Divide(2, 2);

  canvas.cd(1);
  g_dist.Draw("A3");
  g_dist.Draw("LX");

  canvas.cd(2);
  g_vel.Draw("A3");
  g_vel.Draw("LX");

  canvas.cd(3);
  h_dist.Draw();

  canvas.cd(4);
  h_vel.Draw();

  canvas.SaveAs("flock_statistics.png");
}

}  // namespace pf

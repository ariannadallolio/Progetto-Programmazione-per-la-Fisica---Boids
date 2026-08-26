//UTILIZZO ACCUMULATE PER DEV STD

double std_dev_distance_sum =
      std::accumulate(distances.begin(), distances.end(), 0.0,
                      [mean_distance](double accumulator, double d) {
                        double const difference = d - mean_distance;
                        return accumulator += difference * difference;
                      });


//UTILIZZO ACCUMULATE PER COHESION
Position sum = std::accumulate(neighbours.begin(), neighbours.end(), Position{0.0, 0.0},
      [&boids, pos_check, &space](Position const& acc, int m) {
          Position const diff = toroidal_difference(boids[static_cast<std::size_t>(m)].pos, pos_check, space);
          return acc + diff; })// Somma la nuova differenza all'accumulatore                      
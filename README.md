# VRPD

The `VRPD` is the final class project for BMGT831 in University of Maryland, Spring semester, 2015. This project focuses on solving a modified Vehicle Routing Problem (VRP), the VRP with drones (VRPD).

In VRPD, a given number of trucks are to serve each customer which is embeded in a undirected network. Each truck carries a given number of drones and packages. Each customer has to be served either by truck or by drone. The drone moves faster than truck. Due to the battery capacity, the total duration of each flight is limited.

The `VRPD` takes advantage of a well-developed library for traditional VRP problem, the [`VRPH`](http://www.coin-or.org/projects/VRPH.xml), and makes improvements based on it. Generally, `VRPD` modifies the *Clark Wright* method for generating the initial solution, employs the *Simulated Annealing* (SA) algorithm combined with *One Point Move*, *Two Point Move* and *Two Opt* as local searching strategies.

You can find the instances files in `./instance/`. Play with `VRPD` and feel free to contact me for any questions.

## To compile:

Make sure you have installed cmake, and run the scripts:

~~~
cmake cmakelist
make
~~~

The execuable file `VRPD` is generated in ./bin

## Usage

The `VRPD` takes a couple of arguments. For more details, please run

~~~
./bin/vrpd -?
~~~

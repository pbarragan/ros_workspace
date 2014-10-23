#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace KDL;
 
 
int main( int argc, char** argv )
{

  //Creation of the chain:
  KDL::Chain chain;
  chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));
  
  //Creation of the solvers:
  ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
  ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
  ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
  
  //Creation of jntarrays:
  JntArray q(chain.getNrOfJoints());
  JntArray q_init(chain.getNrOfJoints());
  
  //Set destination frame
  //Frame F_dest;
  Vector F_trans = Vector(0.60,0,0);
  Frame F_dest = Frame(F_trans);
  
  int ret = iksolver1.CartToJnt(q_init,F_dest,q);

  std::cout << ret << std::endl;

  return 0;
}
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <math.h>

using namespace std;
typedef double IKReal;
int ikLeft(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, const int max_sol, IKReal* solutions);
int ikRight(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, const int max_sol, IKReal* solutions);

bool legalTest(double legal[7][2],vector<double> sol){
  bool isLegal = true; 
  for(size_t i=0;i<sol.size();i++){
    if(sol[i]<legal[i][0] || sol[i]>legal[i][1]){
      isLegal = false;
    }
  }
  return isLegal;
}

bool safeTest(vector<double> sol){
  return true;
}

vector<double> collectSafe(int n, double solnsKin[], double legal[7][2]){
  vector<double> sols;
  for(size_t i=0;i<n;i++){
    //vector<double>::const_iterator first = solnsKin.begin()+i*7;
    //vector<double>::const_iterator last = solnsKin.begin()+(i+1)*7;
    vector<double> sol;
    sol.assign(solnsKin+i*7,solnsKin+(i+1)*7);
    //is it legal and safe - safe isn't currently implemented (always true)
    if(legalTest(legal,sol) && safeTest(sol)){
      for(size_t j=0;j<sol.size();j++){
	sols.push_back(sol[j]);
      }
    }
  }
	return sols;
}

double angleDiff(double angle1, double angle2){
  double pi = 3.14159265359;
  double angleA = fmod(angle1,(2*pi));
  double angleB = fmod(angle2,(2*pi));
  if(angleA>=angleB){
    return min(angleA-angleB,angleB+(2*pi-angleA));
  }
  else{
    return min(angleB-angleA,angleA+(2*pi-angleB));
  }
}

double solnDist(vector<double> sol1,vector<double> sol2){
  double total = 0;
  for(size_t i=0;i<sol1.size();i++){
    total += fabs(angleDiff(sol1[i],sol2[i])); //DEAL WITH THE WRAP
  }
  return total;
}

vector<double> pr2KinIKfast(double T[4][4], vector<double> current){
  //vector<vector<double>> legal;
  //cout << "i'm in pr2KinIKfast" << endl;

  double legal [7][2] = {{-0.564601836603, 2.1353981634},{-0.3536, 1.2963}, {-0.65, 3.75},
			 {-2.1213, -0.15}, {-3.142592653589793, 3.142592653589793},
			 {-2.0, -0.1}, {-3.142592653589793, 3.142592653589793}};

  //cout << legal[2][1] << endl;
  int nsolnsKin = 10;  
  double rotKin[9];
  double transKin[3];
  double freeKin[1];
  double solnsKin[7*nsolnsKin]; 

  for(size_t i=0;i<3;i++){
    for(size_t j=0;j<3;j++){
      rotKin[i*3+j]=T[i][j];
    }
  }
  for(size_t i=0;i<3;i++){
    transKin[i]=T[i][3];
  }

  double step = 0.01; //XXX TOMAS!
  double lower = legal[2][0];
  double upper = legal[2][1];
  double th0 = current[2];
  vector<double> best;

  int n = 0;

  if(th0>=lower && th0<=upper){
    //cout << "im inside if 1" << endl;

    int nsteps = max((upper-th0)/step,(th0-lower)/step);
    //cout << "upper: " << upper << endl;
    //cout << "lower: " << lower << endl;
    //cout << (upper-th0)/step << endl;
    //cout << (th0-lower)/step << endl;
    //cout << "nsteps: " << nsteps << endl;
    //solver; //XXXTOMAS! add this part after everything compiles ******************************
    vector<double> sols;
    for(size_t i=0;i<nsteps;i++){
      double stepsize = i*step;
      freeKin[0] = th0+stepsize;
      //cout << "i'm inside for loop 1" << endl;
      if(freeKin[0]<=upper){
	n = ikLeft((IKReal*) (&transKin),(IKReal*) (&rotKin),(IKReal*) (&freeKin), 
		   nsolnsKin,(IKReal*) (&solnsKin)); //XXX TOMAS

	//cout << "n: " << n << endl;

	if(n>0){
	  sols = collectSafe(n,solnsKin,legal);
	  if(sols.size()>0){ 
	    break;
	  }
	}
      }
      freeKin[0] = th0-stepsize;
      if(freeKin[0]>=lower){
	n = ikLeft((IKReal*) (&transKin),(IKReal*) (&rotKin),(IKReal*) (&freeKin), 
		   nsolnsKin,(IKReal*) (&solnsKin)); //XXX TOMAS

	//cout << "n: " << n << endl;


	if(n>0){
	  sols = collectSafe(n,solnsKin,legal);
	  if(sols.size()>0){ 
	    break;
	  }
	}
      }
    }
    if(sols.size()>0){
      double bestScore = 1000000;
      //double tempScore = 1000000; //1 MILLION
      for(size_t i=0;i<sols.size()/7;i++){
	vector<double>::const_iterator first = sols.begin()+i*7;
	vector<double>::const_iterator last = sols.begin()+(i+1)*7;
	vector<double> solCurrent(first,last);	
	if(solnDist(current,solCurrent)<bestScore){
	  best = solCurrent;
	}
      }
    }
  }
  return best;
}

/*
int main(){
  double T [4][4] = {{1,0,0,0.60},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  static const double currentTemp[] = {0,0,0,0,0,0,0};
  vector<double> current(currentTemp, currentTemp + sizeof(currentTemp)/sizeof(currentTemp[0]) );
  vector<double> best = pr2KinIKfast(T, current);

  cout << "Best Solution:" << endl;
	 
  for (vector<float>::size_type ii = 0; ii < best.size(); ii++) {
    cout << best[ii] << endl;
  }

  //cout << "Did that work" << endl;

  return 0;
}
*/

/*

def pr2KinIKfast(robot, T, current, safeTest=lambda x: True, exact = False):
    legal = [(-0.564601836603, 2.1353981634),
                  (-0.3536, 1.2963), (-0.65, 3.75),
                  (-2.1213, -0.15), (-3.142592653589793, 3.142592653589793),
                  (-2.0, -0.1), (-3.142592653589793, 3.142592653589793)]
    def legalTest(sol):
        for qp, qr in zip(sol, legal):
            if not qr[0] <= qp <= qr[1]:
                # print 'IKfast illegal angle', qp, qr
                return False
        return True
    def collectSafe(n):
        sols = []
        for i in range(n):
            sol = solnsKin[i*7 : (i+1)*7]
            if sol and legalTest(sol):  # inside joint limits
                if safeTest(sol):       # doesn't collide
                    sols.append(sol)
        return sols
    for i in range(3):
        for j in range(3):
            rotKin[i*3+j] = T[i, j]
    for i in range(3):
        transKin[i] = T[i, 3]
    step = glob.IKfastStepSmall if exact else glob.IKfastStep
    lower, upper = legal[2]             # legal has torso removed
    th0 = current[2]
    if not lower <= th0 <= upper: return []
    nsteps = max((upper-th0)/step, (th0-lower)/step)
    solver = ik.ikRight if glob.right else ik.ikLeft
    sols = []
    for i in range(int(nsteps)):
        stepsize = i*step
        freeKin[0] = th0 + stepsize
        if freeKin[0] <= upper:
            n = solver(transKin, rotKin, freeKin, nsolnsKin, solnsKin)
            # print 'th', th0 + stepsize, 'n', n
            if n > 0:
                sols = collectSafe(n)
                if sols: break
        freeKin[0] = th0 - stepsize
        if freeKin[0] >= lower:
            n = solver(transKin, rotKin, freeKin, nsolnsKin, solnsKin)
            # print 'th', th0 - stepsize, 'n', n
            if n > 0:
                sols = collectSafe(n)
                if sols: break
    # print 'IKFast sols', sols
    best = util.argmax(sols, lambda s: -solnDist(current, s)) if sols else None
    if not best: return None
    for i in range(7): jtKin[i] = best[i]
    # print 'best', best
    return best

def solnDist(sol1, sol2):
    total = 0.0
    for (th1, th2) in zip(sol1, sol2):
        total += abs(util.angleDiff(th1, th2))
    return total

*/

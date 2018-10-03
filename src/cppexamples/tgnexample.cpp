#include <vector>
#include <string>
#include <openrave-core.h>
#include <openrave/robot.h>

using namespace OpenRAVE;
using std::vector;
using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
  
  OpenRAVE::RaveInitialize(true);
  OpenRAVE::EnvironmentBasePtr penv = OpenRAVE::RaveCreateEnvironment();
  // penv->SetDebugLevel(OpenRAVE::Level_Fatal);

  //string robotfile = string(argv[1]);
  penv->Load("man1.zae");

  vector<RobotBasePtr> vrobots;
  penv->GetRobots(vrobots);
  RobotBasePtr probot = vrobots[0];
  vector<RobotBase::ManipulatorPtr> vmanips = probot->GetManipulators();

  RobotBase::ManipulatorPtr pmanip = vmanips[0];
  int baselink = pmanip->GetBase()->GetIndex(), 
    eelink = pmanip->GetEndEffector()->GetIndex();
  cout << "baselink = " << baselink << ", eelink = " << eelink << endl;

  vector<KinBody::JointPtr> vjoints;
  vector<KinBody::LinkPtr> vlinks;
  probot->GetChain(baselink, eelink, vjoints);
  probot->GetChain(baselink, eelink, vlinks);
  
  OpenRAVE::RaveDestroy();
  return 0;
}

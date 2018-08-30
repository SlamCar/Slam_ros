

class Controller
{
  public:
    virtual double getInput() = 0;  
}

class CarController : public Controller
{
  public:
    virtual double getInput(){};
    virtual msgs::CmdVel getCommandVelocity() = 0;

}
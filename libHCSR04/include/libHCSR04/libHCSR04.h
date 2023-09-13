#ifndef DEF_SONAR
#define DEF_SONAR

class HCSR04
{
  public:
    HCSR04(int trigger, int echo);
    // void init();
    double distance(int timeout);
    double speed(int timeout);

  private:
    void recordPulseLength();
    int trigger;
    int echo;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;

    double start_distance;
    double end_distance;
    double speedMS;
};

#endif
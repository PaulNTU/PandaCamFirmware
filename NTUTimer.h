#ifndef __NTUTIMER_H__

#define __NTUTIMER_H__
class NTUTimer
{
  private:

    int countDown = 0;
    int originalTime;
    unsigned long lastCallTime;
    bool autoReset = true;

  public:
    NTUTimer(bool autoReset)
    {
      this->autoReset = autoReset;
    }

    void Start(int timeSpan);
    bool Update();
    void Reset();
};

#endif
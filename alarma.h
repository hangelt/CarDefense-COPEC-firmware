#ifndef ALARMA_H_
#define ALARMA_H_

class Alarma 
{
  public:
    Alarma(int nA, unsigned long ts, bool sStatus = false)
    {
      this->numAlarma = nA;
      this->timestamp = ts;
      this->sendStatus = sStatus;
    }

    Alarma()
    {
      this->numAlarma = 0;
      this->timestamp = 0;
      this->sendStatus = true;
    }

    int numAlarma;
    unsigned long timestamp;
    bool sendStatus;
};

#endif  //ALARMA_H_

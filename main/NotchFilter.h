class NotchFilter {
  private:
    float c1;
    float c2;
    float c3;
    float c4;
    float c5;
    float c6;
    float x0;
    float x1;
    float x2;
    float y0;
    float y1;
    float y2;
  public:
    NotchFilter(float w0=0.0, float ww=0.0, float freq=0.0);
    float next(float val);
};
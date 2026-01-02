int movingaverage(int sample) {
  static int array[12] = {0};
  static int arrayPointer = 0;

  array[arrayPointer] = sample;
  arrayPointer = (arrayPointer + 1) % 12;

  long sum = 0;
  for (int i = 0; i < 12; i++) sum += array[i];

  return (int)(sum / 12);
}

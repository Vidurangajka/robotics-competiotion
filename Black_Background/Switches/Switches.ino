void switch() {
  // Read switch states
  int s1 = digitalRead(switch1) == HIGH ? 0 : 1; // Convert to binary logic
  int s2 = digitalRead(switch2) == HIGH ? 0 : 1;
  int s3 = digitalRead(switch3) == HIGH ? 0 : 1;

  // Determine state based on switches
  if (s1 == 0 && s2 == 0 && s3 == 0) code_000();
  else if (s1 == 0 && s2 == 0 && s3 == 1) code_001();
  else if (s1 == 0 && s2 == 1 && s3 == 0) code_010();
  else if (s1 == 0 && s2 == 1 && s3 == 1) code_011();
  else if (s1 == 1 && s2 == 0 && s3 == 0) code_100();
  else if (s1 == 1 && s2 == 0 && s3 == 1) code_101();
  else if (s1 == 1 && s2 == 1 && s3 == 0) code_110();
  else if (s1 == 1 && s2 == 1 && s3 == 1) code_111();

  return 
}
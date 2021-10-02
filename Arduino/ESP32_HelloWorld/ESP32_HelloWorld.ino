int a = 2;
int i = 1;
void setup() {

  // put your setup code here, to run once:
Serial.begin(115200);

}

void loop() {

while (i < 2)
  {  
    i++;
    foo(a);
    a = 1;
  }
}

void foo (int b)
{
  Serial.print(b);
}

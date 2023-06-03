
//Struktur $P %val|I %val |D %val#
//         $ = START
//         P = "Proportional"
//            %Tal-deklarering
//             Tallet 
//                |NÆSTE VÆRDI
//                 I="Integral"   
//                  %-Tal deklarering
//                   Tallet
//                        |NÆSTE VÆRDI
//                         D = "Derivative"
//                           %-Tal deklarering
//                            Tallet
//                             # END
// $ = 1 byte, P = 1 byte, % = 1 byte, tal = 1 byte, | = 1 byte.....

//char txMessage[5] = "Test\n";
char txMessage[] = "$P %3 |I %8 |D %2#";


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  Serial.write(txMessage, 19);//(txMessage,5);
}

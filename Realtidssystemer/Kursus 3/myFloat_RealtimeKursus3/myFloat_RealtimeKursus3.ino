
/*
  Low level precision floating point lib
  Author: Henrik Schiøler

*/

//low precision floating pt type
typedef struct myfloat {
  signed char mantissa;
  signed char exponent;
} myfloat_type;


//convert from double to low precision type
void doub2mydouble(double arg, myfloat_type *res) {
  int exponent;
  double temp;

  exponent = ceil(log(abs(arg)) / log(2));  //base 2 logarithm
  temp = arg * pow(2, 7 - exponent);
  res->mantissa = (signed char)temp;
  res->exponent = exponent - 7;
}

//convert from low precision type to double
double myfloat2double(myfloat_type *arg1) {
  double res = (double)(arg1->mantissa) * pow(2, arg1->exponent);
  return res;
}

//multiply to low precision types
void mult_float(myfloat_type *arg1, myfloat_type *arg2, myfloat_type *result) {
  int temp;
  unsigned char sign;

  sign = 0x80 & ((unsigned char)arg1->mantissa ^ (unsigned char)arg2->mantissa);  //find sign of result

  char i = 0;
  temp = (int)(arg1->mantissa) * (int)(arg2->mantissa);

  temp = temp & 0x7f00;  //take away sign from product

  while (abs(temp) > 128) {
    i++;
    temp = temp >> 1;
  }

  result->mantissa = (unsigned char)temp;

  result->mantissa = result->mantissa | sign;  //add recorded sign

  result->exponent = arg1->exponent + arg2->exponent + i;
}

//100x3 type array to hold the different data types.
double da[100][3];


double inter = 0;
myfloat_type res;
float mRel = 0;
float mRel2 = 0;
float sumDa = 0;
float sumMda = 0;
int run = 0;

void setup() {
  delay(500);
  double a = 4.0;
  myfloat_type mda[100];


  Serial.begin(115200);

  doub2mydouble(a, &res);
  Serial.println(res.mantissa);
  Serial.println(res.exponent);


  for (int i = 0; i < 100; i++) {
    da[i][0] = random(-5, 5);
  }
}

//da array, [x][0] holds the normal double value
//da array, [x][1] hold the myfloat value

void loop() {
  for (int i = 0; i < 100; i++) {
    doub2mydouble(da[i][0], &res); 
    da[i][1] = res.mantissa * pow(2, res.exponent);   //This is used together with doub2mydouble for the myfloat conversion.
    da[i][2] = ((da[i][1] - da[i][0])/da[i][0])*100;  //Mean Relative error for the individual pairs
    mRel += ((da[i][1] - da[i][0]));  //Summed absolute error, used to calculate the totalt mean relative error.
    sumDa += da[i][0];  //Sum of the true values, used for calculate total mean relative error.
    Serial.print(da[i][0],5);   //Prints true value, low resolution value, and error
    Serial.print("    ||    ");
    Serial.print(da[i][1],5);
    Serial.print("    ||    ");
    Serial.print(da[i][2],5);
    Serial.print("    ||    ");
    da[i][0] = da[i][0] * da[i][0]; //Calculates the true squared value.
    da[i][1] = da[i][1] * da[i][1]; //Calculates the low-res squared value.
    sumMda += da[i][0]; //sum up true squared value for total mean relative error.
    da[i][2] = ((da[i][1] - da[i][0]) / da[i][0]) * 100;  //calculate the indivivual mean relative error of squared value.
    mRel2 += ((da[i][1] - da[i][0])); //sum up absoulte squared error.
    Serial.print(da[i][0],5);   //prints the true squared value.
    Serial.print("    ||    "); //prints the low-res squared value.
    Serial.print(da[i][1],5);
    Serial.print("    ||    "); // Prints the individual mean relative error for squared values.
    Serial.print(da[i][2],5);
    Serial.println("");
  }
  mRel = (mRel/sumDa)*100;    //Caluclate total mean
  mRel2 = (mRel2/sumMda)*100; //calculate total squared mean
  Serial.println(mRel);
  Serial.println(mRel2);
  mRel = 0;
  mRel2 = 0;
  delay(50000);
}


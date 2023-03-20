#include <fstream>
#include <string>
#include <sstream>
//#include <Arduino.h>
#include <M5Stack.h>
#include <vector>
//#include <iostream>
//#include <numeric>
//#include <cmath>
//#include <algorithm>
//#include <map>
//#include <ArduinoEigen.h>
//#include <HTTPClient.h>
//#include <WiFi.h>
#include <WiFiMulti.h>

using namespace std;

const int N_WEIGHTS = 100;
const int N_FEATURES = 118;
int list_count = 0;

bool execute = false;
bool clear_screen = false;

float imu_data[6];
float w_mtx[N_WEIGHTS][N_FEATURES];
int lbls[N_WEIGHTS];

vector<float> gyroXv;
vector<float> gyroYv;
vector<float> gyroZv;
vector<float> accXv;
vector<float> accYv;
vector<float> accZv;
vector<float> timev;

WiFiMulti wifiMulti;

string menu_items[1] = {"testing SOM online preproc"};

int act_count = 1; // activity integer value

string vector_to_string(vector<float> vec) {
    //ostringstream stream;
    string tmp = to_string(vec[0]);
    //stream << vec[0];
    for (int i = 1; i < vec.size(); i++) {
    tmp += "_" + to_string(vec[i]);
    //stream << tmp;
    }
    return tmp;
}

int winner_index(vector<float>& sample) {
    int winner = 0;
    float min_dist = 3.402823466e+38;
    for (int i = 0; i < N_WEIGHTS; i++) {
        float dist = 0;
        for (int j = 0; j < N_FEATURES; j++) {
            /* M5.Lcd.setCursor(0, 0);
            M5.Lcd.printf("sample: %f, j: %d", sample[j], j);
            M5.Lcd.setCursor(0, 20);
            M5.Lcd.printf("w_mtx: %f, j: %d", w_mtx[i][j], j);
            delay(1000); */
            dist += fabs(sample[j] - w_mtx[i][j]);
        }
        
        if (dist < min_dist) {
            min_dist = dist;
            winner = i;
        }
    }
    return winner;
}

vector<float> split(const String& s, char delimiter) {
    vector<float> tokens;
    string token;
    stringstream stream(s.c_str());
    while (getline(stream, token, delimiter)) {
        tokens.push_back(stof(token));
    }
    return tokens;
}

// MAIN PROGRAM
void setup() {
    M5.begin();        // Init M5Core.  初始化 M5Core
    M5.Power.begin();  // Init Power module.  初始化电源
    
    wifiMulti.addAP(
        "wifi_name",
        "wifi_pass");

    M5.IMU.Init();  // Init IMU sensor.  初始化惯性传感器

    M5.Lcd.fillScreen(BLACK);  // Set the screen background color to black.
                               // 设置屏幕背景色为黑色
    M5.Lcd.setTextColor(
        GREEN, BLACK);  // Sets the foreground color and background color of the
                        // displayed text.  设置显示文本的前景颜色和背景颜色
    M5.Lcd.setTextSize(2);  // Set the font size.  设置字体大小

    if (N_WEIGHTS == 100)
    {
        File file = SD.open("/weights_lst_avg_bal_10.txt");

        int row = 0;
        int col = 0;
        String line;
        while(file.available()){
            char c = file.read();
            if(c == '\n'){
                //rows = row + 1;
                //cols = max(cols, col);
                col = 0;
                row++;
                line = "";
            }
            else{
                line += c;
                if(c == ' '){
                    w_mtx[row][col] = line.toFloat();
                    col++;
                    line = "";
                }
            }
        }
        file.close();

        file = SD.open("/map_lst_bal_10.txt");

        row = 0;
        //int col = 0;
        line = "";
        while(file.available()){
            char c = file.read();
            if(c == '\n'){
                //rows = row + 1;
                //cols = max(cols, col);
                //col = 0;
                row++;
                line = "";
                //delay(1000);
            }
            else{
                line += c;
                //M5.Lcd.setCursor(0, 0);
                //M5.Lcd.printf("val: %d", line.toInt());
                lbls[row] = line.toInt();
                // line = "";
            }
        }
        file.close();
    }
    else {
        File file;
        int row = 0;
        int col = 0;
        String line;
        for (int i = 1; i <= 10; i++)
        {
            string path = "/weights_lst_avg_bal_17_pt" + to_string(i) + ".txt";
            file = SD.open(path.c_str());

            while(file.available()){
                char c = file.read();
                if(c == '\n'){
                    //rows = row + 1;
                    //cols = max(cols, col);
                    col = 0;
                    row++;
                    line = "";
                }
                else{
                    line += c;
                    if(c == ' '){
                        w_mtx[row][col] = line.toFloat();
                        col++;
                        line = "";
                    }
                }
            }
            file.close();

            
        }
        file = SD.open("/map_lst_bal_17.txt");

        row = 0;
        //int col = 0;
        line = "";
        while(file.available()){
            char c = file.read();
            if(c == '\n'){
                //rows = row + 1;
                //cols = max(cols, col);
                //col = 0;
                row++;
                line = "";
                //delay(1000);
            }
            else{
                line += c;
                //M5.Lcd.setCursor(0, 0);
                //M5.Lcd.printf("val: %d", line.toInt());
                lbls[row] = line.toInt();
                // line = "";
            }
        }
        file.close();
        
    }
    
    

    int count = 0;

    while (wifiMulti.run() != WL_CONNECTED) {
        M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        if (count == 0)
        {
            M5.Lcd.printf("Connecting to wifi");
            count++;
        }
        else if (count == 1)
        {
            M5.Lcd.printf("Connecting to wifi.");
            count++;
        }
        else if (count == 2)
        {
            M5.Lcd.printf("Connecting to wifi..");
            count++;
        }
        else if (count == 3)
        {
            M5.Lcd.printf("Connecting to wifi...");
            count = 0;
        }
        delay(10);
    }
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Connected to wifi");
    delay(1000);
    
}

void loop(){
    if (!execute)
    {
        clear_screen = false;
        M5.update();  // Read the press state of the key.  读取按键 A, B, C 的状态
        
        //M5.Lcd.clear();
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("Start");
        //M5.Lcd.setCursor(0, 25);
        //M5.Lcd.printf("activity:");
        // set selcted activity
        M5.Lcd.setCursor(0, 25);
        M5.Lcd.printf("- %s", menu_items[act_count - 1].c_str());
        /* M5.Lcd.setCursor(0, 75);
        M5.Lcd.printf("number of records to record %d", n_records[act_count - 1]); */
        // set btn A label
        M5.Lcd.setCursor(41, 220);
        M5.Lcd.printf("Prev");
        // set btn B label
        M5.Lcd.setCursor(129, 220);
        M5.Lcd.printf("Start");
        // set btn C label
        M5.Lcd.setCursor(229, 220);
        M5.Lcd.printf("Next");

        // set btn Prev
        /* if (M5.BtnA.wasPressed())
        {
            if (act_count == 1)
            {
                act_count = 9;
            } 
            else
            {
                act_count--;
            }
            
            clear_screen = true;
        } */

        // set btn Start
        if (M5.BtnB.wasPressed())
        {
            execute = true;
            clear_screen = true;
        }

        // set btn Next
        /* if (M5.BtnC.wasPressed())
        {
            if (act_count == 9)
            {
                act_count = 1;
            } 
            else
            {
                act_count++;
            }
            
            clear_screen = true;
        } */
    } else if (execute) // branch executed if start button was pressed
    {
        clear_screen = false;
        
        if (act_count == 1)
        {
            // get sensor data
            M5.IMU.getGyroData(&imu_data[0], &imu_data[1], &imu_data[2]);
            M5.IMU.getAccelData(&imu_data[3], &imu_data[4], &imu_data[5]);

            M5.Lcd.setCursor(0, 142);
            M5.Lcd.printf("steps: %d ", list_count);
            M5.Lcd.setCursor(0, 162);
            M5.Lcd.printf("Activity: %s", menu_items[act_count - 1].c_str());
            M5.Lcd.setCursor(0, 182);
            M5.Lcd.printf("act_count: %d", act_count);
            list_count +=1;

            if (list_count <= 129)
            {
                if (list_count == 129){
                    list_count = 0;
                    vector<float> features;                    
                    /* if (act_count == 9)
                    {
                        String response;
                        if ((wifiMulti.run() == WL_CONNECTED)){
                            // Change the URL with the target server address
                            HTTPClient http;
                            string gx = vector_to_string(gyroXv);
                            gyroXv.clear();
                            string gy = vector_to_string(gyroYv);
                            gyroYv.clear();
                            string gz = vector_to_string(gyroZv);
                            gyroZv.clear();
                            string ax = vector_to_string(accXv);
                            accXv.clear();
                            string ay = vector_to_string(accYv);
                            accYv.clear();
                            string az = vector_to_string(accZv);
                            accZv.clear();
                            string url;
                            if (act_count == 9){
                                url  = "http://scianso.pythonanywhere.com/som?gx=" + gx + "&gy=" + gy + "&gz=" + gz + "&ax=" + ax + "&ay=" + ay + "&az=" + az;
                            }
                            if (act_count == 7){
                                url  = "http://scianso.pythonanywhere.com/kmeans?gx=" + gx + "&gy=" + gy + "&gz=" + gz + "&ax=" + ax + "&ay=" + ay + "&az=" + az;
                            }
                            http.begin(url.c_str());
                            //http.addHeader("Content-Type", "application/x-www-form-urlencoded");
                            M5.Lcd.clear();
                            int httpCode = http.GET();
                            if (httpCode > 0) {
                                response = http.getString();
                                //M5.Lcd.setCursor(0, 0);
                                //M5.Lcd.printf("response: %s", response.c_str());
                            } else {
                                M5.Lcd.clear();
                                M5.Lcd.setCursor(0, 162);
                                M5.Lcd.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
                            }
                            http.end();
                            //delay(5000);
                            //M5.Lcd.clear();                  
                        }
                        features = split(response, ' ');
                        
                        int win_idx = winner_index(features);
                        int win_lbl = lbls[win_idx];
                        string win_act = menu_items[win_lbl];
                        M5.Lcd.setCursor(0, 202);
                        M5.Lcd.printf("winner index: %d", win_idx);
                        M5.Lcd.setCursor(0, 222);
                        M5.Lcd.printf("pred. act.: %s", win_act.c_str());
                        delay(5000);
                    } */
                                        
                }
                else
                {
                    gyroXv.push_back(imu_data[0]);
                    gyroYv.push_back(imu_data[1]);
                    gyroZv.push_back(imu_data[2]);
                    accXv.push_back(imu_data[3]);
                    accYv.push_back(imu_data[4]);
                    accZv.push_back(imu_data[5]);
                    if (list_count == 1){
                        timev.push_back(0.00);
                    } else {
                        timev.push_back((1/41.29) * (list_count - 1));
                    }
                    delay(10);
                }
                /* if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || M5.BtnC.wasPressed()){
                    execute = false;
                } */
            }
        }
    //delay(20);
    }
    if (clear_screen)
    {
        M5.Lcd.clear();
    }
}

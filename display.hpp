#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <TFT_eSPI.h>
#include <deque>

#define DISPLAY_MEM __not_in_flash("display")

TFT_eSPI DISPLAY_MEM tft = TFT_eSPI();  // Invoke custom library


class display {
public:
    void setup() {
        tft.begin();
        tft.setRotation(1);
        tft.fillScreen(TFT_BLACK);
        tft.setFreeFont(&FreeSans9pt7b);
        tft.setTextColor(0xFC9F);

    }
    
    void post(String str) {
        redraw = true;
        lines.push_back(str);
        if(lines.size() > 11) {
            lines.pop_front();
        }
    }

    void update() {
        if (redraw) {
            redraw = false;
            tft.fillScreen(TFT_BLACK);
            constexpr int32_t lineheight = 20;
            for(size_t i=0; i < lines.size(); i++) {
                tft.drawString(lines[i].c_str(), 10,i*lineheight);
            }
        }
    }
private:
    std::deque<String> lines;
    bool redraw=false;
};

#endif
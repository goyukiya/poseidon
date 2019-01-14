---
layout: page
title: "Hardware"
group: navigation
---

{% include JB/setup %}
### Open source syringe pumps and Raspberry Pi microscope
![mvimg_20180111_222424](https://user-images.githubusercontent.com/12504176/34991157-69e99c68-fa7d-11e7-8a77-660660820391.jpg)

### All the parts for 3 pumps and a microscope [cost under $400](https://docs.google.com/spreadsheets/d/e/2PACX-1vSY0apQMOMEC040cuPamMla8yvhqwZEs39H3IEm0rRVuf6EW1HUUKMYhD6gZyLmJnDAxj-zRwVM9L6G/pubhtml)
![mvimg_20180111_211022](https://user-images.githubusercontent.com/12504176/34991323-0a6b41aa-fa7e-11e7-8e57-fbb78b54cc67.jpg)


&nbsp;

&nbsp;

## Syringe pumps

<div style="position:relative;padding-top:56.25%;">
  <iframe src="https://www.youtube.com/embed/7YSiO6usR1M" frameborder="0" allowfullscreen
    style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>

&nbsp;

&nbsp;

## Arduino CNC shield

<div style="position:relative;padding-top:56.25%;">
  <iframe src="https://www.youtube.com/embed/Xl02fsRCJ7U" frameborder="0" allowfullscreen
    style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>

We use the [Arduino CNC shield](http://wiki.keyestudio.com/index.php/Ks0095_Arduino_CNC_Kit_/_CNC_Shield_V3.0_%2Bkeyestudio_Uno_R3%2B4pcs_a4988_Driver_/_GRBL_Compatible) to allow for up to three pumps can be controlled from a computer or from the Rapsberry Pi microscope.
The software is configured to run the stepper motors with 200 steps per revolution at 1/32 microstepping, which translates to 6400 steps per rotation. To configure this, it is necessary to add 3 jumpers between the M0, M1 and M2 pins of the Arduino CNC shield below. The Arduino assembly instruction video shows how to to this. More information about microstepping can be found in the product page for the [DRV8825 Stepper Motor Driver](https://www.pololu.com/product/2133), which is used by the CNC shield. 

&nbsp;

&nbsp;

## Raspberry Pi and screen 

<div style="position:relative;padding-top:56.25%;">
  <iframe src="https://www.youtube.com/embed/g3pXNY8snOg" frameborder="0" allowfullscreen
    style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>

&nbsp;

&nbsp;

## Microscope 

<div style="position:relative;padding-top:56.25%;">
  <iframe src="https://www.youtube.com/embed/Szg-vjukonA" frameborder="0" allowfullscreen
    style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>
 
 

#### License

poseidon is distributed under the [BSD 2-Clause License](https://github.com/pachterlab/poseidon/blob/release/LICENSE)

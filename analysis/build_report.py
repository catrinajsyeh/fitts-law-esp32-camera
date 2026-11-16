"""Write the configuration part of the report. No trial results yet.

  python analysis/build_report.py
"""
from __future__ import annotations

import os

from fpdf import FPDF

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "analysis", "out")

PARAGRAPHS = [
    (
        "Board and how we find the hand",
        "We are using an ESP32-S3 with an OV3660 camera. It plugs in as COM3 through a CH343 USB serial "
        "adapter. The board has 8 MB of flash and no PSRAM, and the camera pins are the ESP32-S3-EYE set. "
        "Everything that decides where the hand is runs on the ESP32. The laptop does not get the camera "
        "image during a trial and does not do any of the recognition.",
    ),
    (
        "",
        "The camera lies flat and looks straight up at a plain white ceiling. We keep lamps out of the "
        "picture. The person puts an elbow on the desk and holds their hand over the lens, palm toward "
        "the camera, high enough that ceiling still shows around the hand. If the hand fills the whole "
        "frame, a left-right sweep barely moves the cursor, so that height matters.",
    ),
    (
        "",
        "The board grabs 160 by 120 grayscale frames, subtracts a picture of the empty ceiling, and "
        "keeps the biggest blob as the hand. The cursor is not the middle of the whole arm. It is the "
        "side-to-side center of the far end of the hand, the part away from the elbow. Before the first "
        "person we do one open-hand sweep for 6 seconds and save it. After that the mapping is absolute: "
        "where the hand sits between the left and right ends of that sweep is where the cursor sits on "
        "the screen. We mirror it so the cursor moves the same way the hand does. A light filter keeps "
        "it from jittering when the hand is still. If one frame jumps more than about a quarter of the "
        "range, that frame is thrown out.",
    ),
    (
        "Grab and release",
        "A fist is the button down and opening the hand is the button up. We do not count fingers. We "
        "look at how solid the end of the hand is. A grab starts when that solidity is at least 76, it "
        "stays grabbed at 70 or above, and it lets go at 60 or below. Anything in between waits until "
        "3 frames agree, so the click is a little late, on the order of an eighth of a second. A blob "
        "smaller than 100 pixels cannot start a grab. Fingertips just entering the picture look solid "
        "and would click by accident.",
    ),
    (
        "",
        "Opening the hand moves the point we track. The first time we tried this, the orange bar jumped "
        "off the blue target at the moment the hand opened, because the cursor moved before the release "
        "was counted. Now, once the fist starts to loosen, the bar stays where the fist was and the drop "
        "happens there.",
    ),
    (
        "What the laptop gets, and how a trial is scored",
        "The board talks to the laptop over the USB cable at 115200 baud. It sends cursor positions, "
        "button presses and releases, and whether the hand is still tracked. It does not send pictures "
        "or hand landmarks during practice or the recorded trials. The setup screen does show a small "
        "camera view in the corner so we can see if the hand is in frame. That view turns off when "
        "practice starts, and the laptop never uses it to find the hand.",
    ),
    (
        "",
        "The laptop draws the cursor live, an orange bar you drag, and a blue target. The two bars are "
        "the same size and their centers are at the same height, so the task is only left and right. "
        "Movement time starts when the fist closes on the orange bar and stops when the hand opens. We "
        "use the ESP32's own millisecond clock for that, so delay on the USB cable is not added into "
        "the time.",
    ),
    (
        "",
        "A drop counts only when the orange bar covers more than 80 percent of the blue bar. Exactly "
        "80 percent is a miss. The screen says HIT or MISS. A miss is saved and that same target comes "
        "back, so it does not count as one of the 10 successes.",
    ),
    (
        "Sizes, the room, and how we run people",
        "The three widths are 120, 240, and 480 pixels. The three distances are 480, 1080, and 1920 "
        "pixels. That set fits on the 2560 by 1440 monitor we are using. A smaller screen would shrink "
        "the distances, so we are keeping this monitor, this camera spot, and the room lights the same "
        "for every person. We are not moving the camera or turning lights on and off between people.",
    ),
    (
        "",
        "Each person gets 5 practice drops on the middle target, distance 1080 and width 240. Practice "
        "is just so they learn the fist. It is not part of the plot. Then we press space and the "
        "recorded part starts. There are 9 width-and-distance pairs, 10 successful drops each, 90 per "
        "person. The order is a balanced Latin square picked from the participant number, so not "
        "everyone starts on the easy ones. We tell everyone the same thing: put the cursor on the "
        "orange bar, make a fist, drag it onto the blue bar, and open your hand. Go fast, but the drop "
        "still has to land.",
    ),
    (
        "What still goes wrong",
        "If the hand leaves the picture, the cursor stays where it was instead of jumping. If you were "
        "holding the bar, it lets go after about half a second so it is not stuck to the cursor forever. "
        "A loose fist sometimes does not register, and the person has to close harder. The 3-frame check "
        "adds a little lag on both the grab and the release. And if the hand is too low, the ceiling "
        "disappears from the setup picture and the sweep stops reaching both edges of the screen.",
    ),
]


def build():
    os.makedirs(OUT, exist_ok=True)
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=18)
    pdf.add_page()
    pdf.set_font("Helvetica", "B", 16)
    pdf.multi_cell(0, 8, "Fitts' law setup", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(3)
    pdf.set_font("Helvetica", "B", 13)
    pdf.multi_cell(0, 7, "Results", new_x="LMARGIN", new_y="NEXT")
    pdf.set_font("Helvetica", size=11)
    pdf.multi_cell(
        0, 6,
        "Plot goes here after the recorded trials. Successful drops only. X-axis is index of "
        "difficulty (bits). Y-axis is movement time (ms). One color per person, with a legend.",
        new_x="LMARGIN", new_y="NEXT",
    )
    pdf.ln(2)
    top = pdf.get_y()
    pdf.rect(pdf.l_margin, top, pdf.epw, 90)
    pdf.set_y(top + 40)
    pdf.set_font("Helvetica", "I", 11)
    pdf.cell(0, 6, "leave this blank for the plot", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.set_y(top + 94)
    pdf.set_font("Helvetica", size=11)
    pdf.multi_cell(0, 6, "Does movement time go up as the target gets harder?", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(8)
    pdf.multi_cell(0, 6, "Unsuccessful attempts, counted here and not on the plot:", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(10)
    pdf.set_font("Helvetica", size=11)
    for title, text in PARAGRAPHS:
        if title:
            pdf.ln(2)
            pdf.set_font("Helvetica", "B", 13)
            pdf.multi_cell(0, 7, title, new_x="LMARGIN", new_y="NEXT")
            pdf.set_font("Helvetica", size=11)
        pdf.multi_cell(0, 6, text, new_x="LMARGIN", new_y="NEXT")
        pdf.ln(1)
    pdf.ln(3)
    pdf.set_font("Helvetica", "B", 13)
    pdf.multi_cell(0, 7, "To do", new_x="LMARGIN", new_y="NEXT")
    pdf.set_font("Helvetica", size=11)
    for item in (
        "Run three people, P01 P02 and P03, on this same screen with the same camera and lights.",
        "Each person: 5 practice drops, then 10 successful recorded drops on each of the 9 targets (90).",
        "Put the plot in the blank box. Successful recorded trials only. Label the axes with the units, and add the legend.",
        "Write whether the times follow Fitts' law, meaning slower when the index of difficulty is higher.",
        "Write the unsuccessful counts under the plot. Do not put the misses on the plot.",
        "Put our names on the PDF and zip the code with the csv logs.",
    ):
        pdf.multi_cell(0, 6, "- " + item, new_x="LMARGIN", new_y="NEXT")
    out = os.path.join(OUT, "Assignment3_FittsLawESP32_YourName.pdf")
    pdf.output(out)
    print(out)


if __name__ == "__main__":
    build()

# Running the Fitts study

You are collecting the user study. Each person does a short practice, then the recorded trials. Use the same camera, the same lights, the same screen, and the same words for every person. Do not change the fist settings or the sweep range between people.

The board is the ESP32-S3 camera on USB port COM3. The camera lies flat and looks straight up at a plain white ceiling. No lamp in the picture. The person rests an elbow on the desk, holds the bare hand above the lens with the palm toward the camera, and moves only the forearm. The hand should be high enough that ceiling shows around it.

## Start the tracker first

Open a terminal in this project folder and run:

```
python analysis/track.py
```

Leave that window open. A page opens at http://127.0.0.1:8765/. It updates every few seconds from the saved logs. This is how you see progress during the study.

## Start one participant

Plug in the board. Close anything else that is using COM3. Stay in this project folder and run:

```
python -u laptop_app/fitts_app.py --pid P01 --port COM3
```

Use `P02` for the second person and `P03` for the third. The study window should open on the large monitor.

Keep the hand out of the picture for a few seconds at the start so the board can learn the empty ceiling. If a hand was already over the lens, press **B** with the hand out, then bring the hand back.

## What to check before any trials

The camera picture is only in the upper-right corner of the setup screen. Look at it, then do these checks in order. Stop at the first one that fails.

1. The small picture shows ceiling around the hand. If it is only skin, the hand is too close. Raise it.
2. Move the open hand slowly left and right. The cursor should move the same way and stay still when the hand stays still. Press **M** if it moves backward.
3. Press **C**, sweep the open hand to both sides for 6 seconds, then press **S**. A full sweep should reach both edges of the screen. Do this once, before the first person, and leave it saved.
4. Make a fist. The cursor dot fills in. Open the hand. The dot becomes a ring. A loose hand, or only the fingertips at the edge of the picture, should not grab.

Read this to every participant, in these words:

> Put the cursor on the orange bar, make a fist to grab it, drag it onto the blue bar, and open your hand to drop it. Work as quickly and accurately as you can. A drop counts only if the orange bar covers more than 80 percent of the blue bar.

## What a session looks like

Press **Enter** to start practice. The camera picture turns off. That is expected. Practice is 5 drops of one medium target. Practice is saved, but it is not part of the results and it is not drawn on the plot.

When the screen says practice is finished, press **Space**. Recording starts. There are 9 targets: 3 widths and 3 distances. Each target continues until that person has 10 successful drops. A miss is saved and the same target comes back. A person is done at 90 successful recorded drops. Press **Space** again at each rest break.

A green **HIT** means the drop counted. A red **MISS** means it did not. The orange and blue bars are the same size and sit at the same height.

Press **Esc** twice to quit. The log is saved.

## What to watch while they are moving

- The grab only starts when the cursor is on the orange bar. A fist made off the bar does not pick it up. They should open the hand, aim, and fist again.
- Opening the hand should drop the block where it is. It should not jump off the blue bar.
- If the hand leaves the picture, the cursor should stay put. A grab should let go within about half a second. It should not stay stuck and it should not jump across the screen.
- Do not press **K** unless you are giving up on that attempt. **K** logs a miss.
- Do not move the camera, the lights, the chair, or the screen between people. Do not press **C** again unless the cursor clearly no longer reaches both edges.

## How to read the tracker

The top of the page is one card per person.

- `0/90 recorded successes` is the number that matters. The assignment needs 90 per person.
- Each box is one target. It shows the width W, the distance D, the index of difficulty in bits, and how many successes that target has. A box is finished at `10/10`. Misses on that target are listed next to the successes.
- The line under the cards says if you still need more people. You need at least 3.

The plot is under that. The x-axis is **Index of difficulty (bits)**. The y-axis is **Movement time (ms)**. One color per person, named in the legend. A point is one successful recorded drop. Practice drops are never on this plot. A harder target (higher on the x-axis) is expected to take longer, so the points should trend upward. Until someone has a recorded success, the plot says that no recorded trials are in yet.

The table is every attempt, newest first.

| Column | Meaning |
|---|---|
| Participant | P01, P02, or P03 |
| Phase | `practice` or `recorded`. Only `recorded` counts toward the 90 |
| Attempt | Number of this drop inside that run |
| D (px) | Distance between the orange bar and the blue bar |
| W (px) | Width of both bars |
| ID (bits) | Index of difficulty, log2(D/W + 1). Bigger means harder |
| Movement time (ms) | Time from the fist grabbing the orange bar until the hand opens |
| Success | `success` counted toward the 10. `miss` did not |
| Overlap | Fraction of the orange bar that covered the blue bar. It must be above 0.80 |
| Log | The file this row was saved in |

## Where the files are

Each run writes two files in the `data` folder:

- `P01_date_time.csv` is the table of attempts
- `P01_date_time_session.json` is the screen size and the order of the targets

After all three people are done, leave those files in `data`. Do not rename them and do not type extra rows into them. The tracker page is the table and the plot. The written setup description is separate and does not include the trial numbers yet.

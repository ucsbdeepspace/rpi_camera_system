"""One-off builder for docs/261119 Phillips led_timing_latency_distribution.pptx.

Summarizes the 2026-11-19 LED-toggle latency characterization: redoing the
old mean/max-only closed-loop LED tests to keep every per-toggle sample,
the two-peak frame-cadence-quantization finding, the self-entrainment
methodology bug in the test loop itself (and the real pass/fail verdict it
flipped once fixed), and the final 4-mode verdict table. See CLAUDE.md's
2026-11-19 entry for the full narrative this deck summarizes.

Matches the visual style of docs/261118 Phillips camera_timing_results.pptx
(extracted directly from that file's XML -- colors, fonts, exact layout
geometry -- rather than guessed) since that's the most recent deck in this
series and the one the audience has most recently seen. Not meant to be a
general reusable tool -- rerun by hand if the deck needs regenerating.

Verified by actually rendering every slide (soffice --headless --convert-to
pdf, then pdftoppm) and inspecting the PNGs -- not just "the script didn't
crash". Found and fixed real layout bugs this way: titles wrapping into the
content below them, and callout boxes sized from guessed text-height
estimates that overflowed into the footer. The callout-overflow class of
bug is why callouts are now anchored to a FIXED position relative to the
footer (CALLOUT_TOP, computed once) instead of computed from whatever
content happens to sit above them on a given slide -- the content above is
sized to fit above that fixed point, not the other way around, so a wrong
height estimate for one thing can no longer collide with the footer.
"""

from pptx import Presentation
from pptx.util import Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE
from PIL import Image

SLIDE_W = Emu(12192000)
SLIDE_H = Emu(6858000)

# ── Palette (extracted from docs/261118 .../camera_timing_results.pptx) ────
NAVY = RGBColor(0x14, 0x20, 0x2B)          # dark bg; title text on light bg
CREAM = RGBColor(0xF6, 0xF5, 0xF1)          # light bg variant 1; headline text on dark bg
LIGHT_BG = RGBColor(0xEB, 0xEE, 0xF1)       # light bg variant 2 (alternated for visual variety)
ACCENT_BLUE = RGBColor(0x8F, 0xB8, 0xF0)    # eyebrow labels on dark bg
TEAL = RGBColor(0x0F, 0x7C, 0x74)           # step-card numbers
BODY_GRAY = RGBColor(0x4A, 0x55, 0x63)      # body/caption text on light bg
FOOTER_GRAY = RGBColor(0x6B, 0x74, 0x80)    # footer citation text
SUBTITLE_ON_DARK = RGBColor(0xB9, 0xC4, 0xCE)
BADGE_FILL = RGBColor(0x2A, 0x37, 0x44)
BADGE_TEXT = RGBColor(0xDD, 0xE3, 0xE8)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
ORANGE = RGBColor(0xE0, 0x9A, 0x4F)         # "open question" callouts

FONT_SANS = "IBM Plex Sans"
FONT_MONO = "IBM Plex Mono"

LEFT = Emu(804672)
CONTENT_W = Emu(10582351)
TITLE_TOP = Emu(658368)
TITLE_H = Emu(548640)            # sized for ONE line at 32pt -- titles below
                                    # are kept short enough to fit it (see
                                    # module docstring: this was a real bug)
CONTENT_TOP = Emu(1554480)       # fixed start-of-content for every slide,
                                    # with enough clearance below TITLE_H
FOOTER_TOP = Emu(6144768)
FOOTER_H = Emu(411480)

CALLOUT_H = Emu(1310640)  # fits 3 wrapped lines of 14pt body text comfortably
                             # -- 1097280 (2-line budget) overflowed on the
                             # first real run with longer callout text,
                             # caught by rendering + inspecting, not assumed
CALLOUT_GAP_ABOVE = Emu(137160)  # minimum gap between content above and the callout
CALLOUT_TOP = Emu(FOOTER_TOP - Emu(173736) - CALLOUT_H)  # fixed, footer-anchored
CONTENT_BOTTOM = Emu(CALLOUT_TOP - CALLOUT_GAP_ABOVE)     # anything above the
                                                             # callout must end
                                                             # by here

RESULTS = "results/"


def _set_bg(slide, color):
    slide.background.fill.solid()
    slide.background.fill.fore_color.rgb = color


def _textbox(slide, left, top, width, height):
    box = slide.shapes.add_textbox(left, top, width, height)
    box.text_frame.word_wrap = True
    return box


def _run(paragraph, text, size, color, bold=False, font=FONT_SANS):
    # paragraph.add_run() always appends a new run -- correct whether the
    # paragraph is fresh (first run) or already has text (e.g. the closing
    # slide's bold-lead + regular-rest two-run items).
    run = paragraph.add_run()
    run.text = text
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.name = font
    run.font.color.rgb = color
    return run


def _simple_text(slide, left, top, width, height, text, size, color, bold=False,
                  font=FONT_SANS, align=PP_ALIGN.LEFT, wrap=True):
    box = _textbox(slide, left, top, width, height)
    box.text_frame.word_wrap = wrap
    p = box.text_frame.paragraphs[0]
    p.alignment = align
    _run(p, text, size, color, bold, font)
    return box


def _rounded_rect(slide, left, top, width, height, fill_color):
    shape = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    shape.fill.solid()
    shape.fill.fore_color.rgb = fill_color
    shape.line.fill.background()
    shape.shadow.inherit = False
    return shape


def _place_image_in_frame(slide, image_path, frame_left, frame_top, frame_w, frame_h, inset):
    """Draws a white rounded-rect frame and places image_path inside it,
    fit to the frame's REAL aspect ratio (read via PIL, not guessed) and
    centered -- constraining only width (as an earlier version of this
    script did) let a picture's auto-scaled height exceed the available
    frame height and overflow into whatever sits below the frame on the
    slide; found by rendering and inspecting, not assumed. Returns the
    frame shape in case a caller wants it."""
    frame = _rounded_rect(slide, frame_left, frame_top, frame_w, frame_h, WHITE)

    avail_w = frame_w - 2 * inset
    avail_h = frame_h - 2 * inset
    img_w, img_h = Image.open(image_path).size
    aspect = img_w / img_h

    if avail_w / aspect <= avail_h:
        pic_w, pic_h = avail_w, Emu(int(avail_w / aspect))
    else:
        pic_h, pic_w = avail_h, Emu(int(avail_h * aspect))

    pic_left = Emu(frame_left + inset + (avail_w - pic_w) // 2)
    pic_top = Emu(frame_top + inset + (avail_h - pic_h) // 2)
    slide.shapes.add_picture(image_path, pic_left, pic_top, width=Emu(pic_w), height=Emu(pic_h))
    return frame


def add_footer(slide, text, color=FOOTER_GRAY):
    _simple_text(slide, LEFT, FOOTER_TOP, CONTENT_W, FOOTER_H, text, 10, color, font=FONT_SANS)


def add_title_bar(slide, title, color=NAVY):
    # wrap=False is deliberate: a wrapped title has nowhere safe to go on
    # these slides (content starts at a fixed CONTENT_TOP right below it) --
    # better to render a visibly-too-long single line (obvious to catch in
    # review) than silently overlap the content below it (not obvious,
    # exactly what happened before this was added).
    _simple_text(slide, LEFT, TITLE_TOP, CONTENT_W, TITLE_H, title, 32, color,
                 bold=True, wrap=False)


def _add_callout(slide, callout):
    eyebrow, body = callout
    _rounded_rect(slide, LEFT, CALLOUT_TOP, CONTENT_W, CALLOUT_H, NAVY)
    inset_l = Emu(292608)
    inset_t = Emu(210312)
    _simple_text(slide, Emu(LEFT + inset_l), Emu(CALLOUT_TOP + inset_t),
                 Emu(CONTENT_W - 2 * inset_l), Emu(228600), eyebrow, 11,
                 ACCENT_BLUE, font=FONT_MONO)
    _simple_text(slide, Emu(LEFT + inset_l), Emu(CALLOUT_TOP + inset_t + 320040),
                 Emu(CONTENT_W - 2 * inset_l), Emu(CALLOUT_H - inset_t - 320040 - Emu(91440)),
                 body, 14, CREAM)


# ── Slide builders ──────────────────────────────────────────────────────


def add_title_slide(prs, eyebrow, headline_lines, subtitle, badges):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, NAVY)

    _simple_text(slide, LEFT, Emu(868680), CONTENT_W, Emu(274320),
                 eyebrow, 12, ACCENT_BLUE, font=FONT_MONO)

    box = _textbox(slide, LEFT, Emu(2011680), CONTENT_W, Emu(1737360))
    for i, line in enumerate(headline_lines):
        p = box.text_frame.paragraphs[0] if i == 0 else box.text_frame.add_paragraph()
        _run(p, line, 54, CREAM, bold=True)

    box2 = _textbox(slide, LEFT, Emu(3840480), Emu(8229600), Emu(822960))
    p2 = box2.text_frame.paragraphs[0]
    _run(p2, subtitle, 20, SUBTITLE_ON_DARK)

    x = LEFT
    gap = Emu(182880)
    badge_h = Emu(384048)
    for text in badges:
        # Width-from-text-length estimate calibrated against the source
        # deck's own real badges (e.g. "I2C link to Nucleo" -> 2072030 EMU,
        # "OV9281 x 2" -> 1252728 EMU) -- ~130000 EMU/char + base padding,
        # generous enough that 14pt text never wraps inside the badge
        # (confirmed by rendering, not just estimated -- see module docstring).
        w = Emu(int(365760 + len(text) * 132080))
        _rounded_rect(slide, x, Emu(5486400), w, badge_h, BADGE_FILL)
        _simple_text(slide, x, Emu(5486400), w, badge_h, text, 14, BADGE_TEXT,
                     align=PP_ALIGN.CENTER, wrap=False)
        x = Emu(x + w + gap)
    return slide


def add_step_slide(prs, title, steps, callout, footer, bg=LIGHT_BG):
    """steps: list of (number_str, heading, body). Exactly 4, matching the
    source deck's step-card slide. Card height is computed to end at
    CONTENT_BOTTOM regardless of CONTENT_TOP/CALLOUT_TOP, so it can never
    collide with the callout below it."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, bg)
    add_title_bar(slide, title)

    card_w = Emu(2450592)
    gap = Emu(256032)
    card_h = Emu(CONTENT_BOTTOM - CONTENT_TOP)
    top = CONTENT_TOP
    x = LEFT
    for num, heading, body in steps:
        _rounded_rect(slide, x, top, card_w, card_h, WHITE)
        inset = Emu(237744)
        _simple_text(slide, Emu(x + inset), Emu(top + 228600), Emu(card_w - 2 * inset),
                     Emu(365760), num, 14, TEAL, bold=True, font=FONT_MONO)
        _simple_text(slide, Emu(x + inset), Emu(top + 640080), Emu(card_w - 2 * inset),
                     Emu(457200), heading, 17, NAVY, bold=True)
        _simple_text(slide, Emu(x + inset), Emu(top + 1325880), Emu(card_w - 2 * inset),
                     Emu(card_h - Emu(1325880) - inset), body, 13, BODY_GRAY)
        x = Emu(x + card_w + gap)

    _add_callout(slide, callout)
    add_footer(slide, footer)
    return slide


def add_image_kpi_slide(prs, title, image_path, kpis, callout, footer, bg=LIGHT_BG):
    """Single image (left, ~74% width) + stacked KPI numbers (right column,
    ~26% width) -- matches the source deck's step-response slide -- plus a
    full-width callout below for the longer mechanism explanation.

    A first version tried to fit that explanation INTO the narrow (~1.9in)
    right column alongside the KPIs, computing its height as "whatever's
    left before CONTENT_BOTTOM" -- found by rendering that this broke two
    ways at once: the narrow column needs many more wrapped lines than a
    naive estimate assumes, and the leftover-height math could even go
    negative, which LibreOffice rendered as text overflowing the slide's
    right edge entirely rather than wrapping. Using the same footer-
    anchored _add_callout() every other slide already uses avoids both --
    full content width instead of a cramped column, and a size this
    function doesn't have to compute itself.
    """
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, bg)
    add_title_bar(slide, title)

    frame_top = CONTENT_TOP
    frame_h = Emu(CONTENT_BOTTOM - CONTENT_TOP)
    frame_w = Emu(7863840)
    _place_image_in_frame(slide, image_path, LEFT, frame_top, frame_w, frame_h, Emu(91440))

    kpi_left = Emu(LEFT + frame_w + Emu(320040))
    kpi_w = Emu(CONTENT_W - frame_w - Emu(320040))
    y = frame_top
    for big, caption in kpis:
        _simple_text(slide, kpi_left, y, kpi_w, Emu(365760), big, 22, NAVY, bold=True, font=FONT_MONO)
        _simple_text(slide, kpi_left, Emu(y + 420624), kpi_w, Emu(594360), caption, 11, BODY_GRAY)
        y = Emu(y + 1052830)

    _add_callout(slide, callout)
    add_footer(slide, footer)
    return slide


def add_dual_image_slide(prs, title, left_image, left_label, right_image, right_label,
                          callout, footer, bg=LIGHT_BG):
    """Two side-by-side image panels with labels above -- for before/after
    comparisons. Not in the source deck verbatim, but built from the same
    visual language (white rounded-rect frames, same margins/footer)."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, bg)
    add_title_bar(slide, title)

    label_h = Emu(274320)
    top = Emu(CONTENT_TOP + label_h + Emu(45720))
    panel_w = Emu((CONTENT_W - Emu(274320)) // 2)
    panel_h = Emu(CONTENT_BOTTOM - top)

    for i, (img, label) in enumerate([(left_image, left_label), (right_image, right_label)]):
        x = Emu(LEFT + i * (panel_w + Emu(274320)))
        _simple_text(slide, x, CONTENT_TOP, panel_w, label_h, label, 13, NAVY,
                     bold=True, wrap=False)
        _place_image_in_frame(slide, img, x, top, panel_w, panel_h, Emu(73152))

    _add_callout(slide, callout)
    add_footer(slide, footer)
    return slide


def add_full_image_slide(prs, title, image_path, callout, footer, bg=LIGHT_BG):
    """The frame itself is sized to the image's own real aspect ratio
    (read via PIL) and centered -- fitting a full-width frame to a
    taller-aspect image by height alone (tried first, see module
    docstring) left a visibly empty gap on one side."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, bg)
    add_title_bar(slide, title)

    inset = Emu(91440)
    max_w = CONTENT_W
    max_h = Emu(CONTENT_BOTTOM - CONTENT_TOP)
    img_w, img_h = Image.open(image_path).size
    aspect = img_w / img_h
    # Fit the PICTURE (inside the inset) within both bounds, then size the
    # frame to match exactly -- so _place_image_in_frame below has no
    # leftover slack to center within, it's already a snug fit.
    avail_w, avail_h = max_w - 2 * inset, max_h - 2 * inset
    if avail_w / aspect <= avail_h:
        pic_w, pic_h = avail_w, Emu(int(avail_w / aspect))
    else:
        pic_h, pic_w = avail_h, Emu(int(avail_h * aspect))
    frame_w, frame_h = Emu(pic_w + 2 * inset), Emu(pic_h + 2 * inset)
    frame_left = Emu(LEFT + (max_w - frame_w) // 2)  # centered if narrower than max_w
    frame_top = CONTENT_TOP

    _place_image_in_frame(slide, image_path, frame_left, frame_top, frame_w, frame_h, inset)

    _add_callout(slide, callout)
    add_footer(slide, footer)
    return slide


def add_closing_slide(prs, title, items, open_question, page_num):
    """items: list of (bold_lead, rest). Matches the source deck's dark
    numbered-takeaways closing slide."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    _set_bg(slide, NAVY)
    add_title_bar(slide, title, color=CREAM)

    top = CONTENT_TOP
    row_h = Emu(850000)  # comfortably fits 3 wrapped lines at 15pt -- 658368
                           # (2-line budget) let one real item's 3rd line
                           # collide with the next item's row, caught by
                           # rendering + inspecting, not assumed
    for i, (lead, rest) in enumerate(items):
        y = Emu(top + i * row_h)
        _simple_text(slide, LEFT, y, Emu(640080), Emu(365760), f"{i+1:02d}", 14,
                     ACCENT_BLUE, font=FONT_MONO)
        box = _textbox(slide, Emu(LEFT + 685800), y, Emu(9896551), Emu(685800))
        p = box.text_frame.paragraphs[0]
        _run(p, lead, 15, CREAM, bold=True)
        if rest:
            _run(p, " " + rest, 15, SUBTITLE_ON_DARK)

    oq_top = Emu(top + len(items) * row_h + Emu(137160))
    _simple_text(slide, LEFT, oq_top, CONTENT_W, Emu(457200), open_question, 13, ORANGE)

    _simple_text(slide, LEFT, FOOTER_TOP, CONTENT_W, FOOTER_H, str(page_num), 10,
                 SUBTITLE_ON_DARK)
    return slide


def main():
    prs = Presentation()
    prs.slide_width = SLIDE_W
    prs.slide_height = SLIDE_H

    add_title_slide(
        prs,
        "LED LATENCY · FULL DISTRIBUTION",
        ["Camera latency,", "the full distribution"],
        "Redone to keep every toggle sample instead of mean and max — "
        "and a timing bias in the test itself changed a real pass/fail verdict.",
        ["GPIO22 LED", "4 raw sizes", "19.5k samples", "Target: 10ms loop"],
    )

    add_step_slide(
        prs,
        "The old test threw away the shape",
        [
            ("01", "Toggle, then wait",
             "Same closed-loop method as before: command the LED, time until the camera confirms it."),
            ("02", "Auto-calibrate",
             "ROI and threshold found automatically from ON/OFF frame differences, not hand-tuned per run."),
            ("03", "Keep every sample",
             "Every toggle's latency saved to disk, not just the mean/std/max this project reported before."),
            ("04", "Histogram, not headline",
             "A real distribution shape, across all 4 validated raw sizes this project has characterized."),
        ],
        ("WHY THIS MATTERS",
         "A mean and a max can both look fine while hiding two very different populations of "
         "outcomes underneath — which is exactly what happened here, twice."),
        "led_timing_histogram_test.py · GPIO22, camera_view_tool.py's new heartbeat pin · 2",
    )

    add_image_kpi_slide(
        prs,
        "Two sharp peaks, not one smooth curve",
        RESULTS + "led_timing_histogram_20261119T212746Z.png",
        [
            ("1.05 ms", "camera frame period at this mode (640×100)"),
            ("~1.1 ms", "gap between the two peaks, this run"),
            ("0 / 3613", "timeouts, out of toggles detected"),
        ],
        ("THE MECHANISM",
         "The camera only delivers a frame every 1.05ms. A toggle either beats that period's "
         "exposure window (fast peak) or just misses it and waits a full extra period (slow "
         "peak). Confirmed by varying the frame period at 3 different rates — the peak gap "
         "tracked it every time."),
        "led_timing_histogram_test.py 10 1050 640x100 · immediate re-command · 3",
    )

    add_dual_image_slide(
        prs,
        "The test loop was timing itself",
        RESULTS + "led_timing_histogram_20261119T213347Z.png",
        "Before: fires right after detection",
        RESULTS + "led_timing_histogram_random_20261119T214129Z.png",
        "After: random delay before each command",
        ("WHAT CHANGED",
         "Firing each command right after detecting the last toggle ties command timing to "
         "the camera's own clock, not a real disturbance's. A random pre-command delay "
         "breaks that link: the double peak dissolves into one smooth curve, and the "
         "measured worst case gets worse, not better."),
        "led_timing_histogram_test.py 10 3400 640x400 [random] · 640×400 @ 3400µs shown · 4",
    )

    add_full_image_slide(
        prs,
        "That fix changed a real verdict",
        RESULTS + "led_timing_histogram_mode_comparison_random.png",
        ("PASS / FAIL AGAINST THE 10 MS BUDGET, REALISTIC TIMING",
         "640×200 and 640×100 (binned ROI) hold up: max 7.51ms and 6.61ms. Full sensor and "
         "stock 640×400 fail — and 640×400's failure (max 15.18ms, was 9.86ms under the "
         "biased test) would have been missed without this fix."),
        "led_timing_histogram_test.py, all 4 modes, random pre-command delay · 5",
    )

    add_closing_slide(
        prs,
        "Where the LED latency test stands",
        [
            ("Only the binned ROI modes are safe:", "640×200 (max 7.51ms) and 640×100 "
             "(max 6.61ms) clear the 10ms budget under realistic timing; full sensor and "
             "stock 640×400 don't."),
            ("The old immediate-recommand test understated the worst case:", "640×400's max "
             "went from a marginal 9.86ms to a clear-fail 15.18ms once command timing was "
             "decorrelated from the camera's clock."),
            ("The two-peak structure is explained, not mysterious:", "camera frame-cadence "
             "quantization — confirmed by varying the frame period and watching the peak gap "
             "track it at 3 different rates."),
            ("camera_view_tool.py now blinks a heartbeat LED on GPIO22,", "the same pin this "
             "whole characterization used, plus the earlier throughput fix "
             "(238Hz → ~440-475Hz) that made a clean high-rate test possible."),
        ],
        "Open: ON→OFF is consistently slower than OFF→ON in every mode (737µs–2141µs, "
        "scales with exposure time) — a real lead, not yet root-caused.",
        6,
    )

    out_path = "docs/261119 Phillips led_timing_latency_distribution.pptx"
    prs.save(out_path)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()

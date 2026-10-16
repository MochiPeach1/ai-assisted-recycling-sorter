"""Generate the Japanese assignment report as a searchable PDF."""

import re
from html import escape
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer


ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "report.md"
OUTPUT = ROOT / "説明資料_PAI最終課題_MochiMomo.pdf"
FONT_NAME = "HeiseiMin-W3"


def make_styles():
    return {
        1: ParagraphStyle(
            "Title",
            fontName=FONT_NAME,
            fontSize=18,
            leading=25,
            textColor=colors.HexColor("#163A54"),
            spaceAfter=10,
            wordWrap="CJK",
        ),
        2: ParagraphStyle(
            "Section",
            fontName=FONT_NAME,
            fontSize=13,
            leading=19,
            textColor=colors.HexColor("#205B78"),
            spaceBefore=5,
            spaceAfter=5,
            wordWrap="CJK",
        ),
        3: ParagraphStyle(
            "Subsection",
            fontName=FONT_NAME,
            fontSize=11,
            leading=16,
            textColor=colors.HexColor("#205B78"),
            spaceBefore=4,
            spaceAfter=4,
            wordWrap="CJK",
        ),
        0: ParagraphStyle(
            "Body",
            fontName=FONT_NAME,
            fontSize=10,
            leading=16,
            alignment=TA_LEFT,
            spaceAfter=5,
            wordWrap="CJK",
        ),
        -1: ParagraphStyle(
            "Bullet",
            fontName=FONT_NAME,
            fontSize=10,
            leading=16,
            leftIndent=12,
            firstLineIndent=-12,
            spaceAfter=3,
            wordWrap="CJK",
        ),
    }


def build_story(source_text, styles):
    story = []
    for source_line in source_text.splitlines():
        line = source_line.strip()
        if not line:
            story.append(Spacer(1, 2 * mm))
            continue

        heading = re.match(r"^(#{1,3})\s+(.*)$", line)
        bullet = re.match(r"^-\s+(.*)$", line)
        if heading:
            level = len(heading.group(1))
            text = heading.group(2)
            style = styles[level]
        elif bullet:
            text = "・" + bullet.group(1)
            style = styles[-1]
        else:
            text = line
            style = styles[0]

        text = re.sub(r"`([^`]*)`", r"\1", text)
        story.append(Paragraph(escape(text), style))
    return story


def draw_footer(canvas, document):
    canvas.saveState()
    canvas.setFont(FONT_NAME, 8)
    canvas.setFillColor(colors.HexColor("#657786"))
    canvas.drawRightString(A4[0] - 18 * mm, 10 * mm, str(document.page))
    canvas.restoreState()


def main():
    pdfmetrics.registerFont(UnicodeCIDFont(FONT_NAME))
    source_text = SOURCE.read_text(encoding="utf-8")
    document = SimpleDocTemplate(
        str(OUTPUT),
        pagesize=A4,
        rightMargin=18 * mm,
        leftMargin=18 * mm,
        topMargin=18 * mm,
        bottomMargin=18 * mm,
        title="説明資料：AI×ロボティクスによる創造的タスクの実現",
        author="Project",
        subject="Physical AI final assignment",
    )
    document.build(
        build_story(source_text, make_styles()),
        onFirstPage=draw_footer,
        onLaterPages=draw_footer,
    )
    print(f"Created {OUTPUT}")


if __name__ == "__main__":
    main()

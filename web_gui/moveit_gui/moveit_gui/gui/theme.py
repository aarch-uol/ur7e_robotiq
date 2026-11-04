"""The visual design system: tokens + one stylesheet. Widgets get a `role` property, not ad-hoc styles.

Colour rules:
  accent   the one primary action on a screen, and the current selection
  ok       healthy state only, never a button
  warn     caution / experimental / direct torque
  danger   real stop, delete and fault only
"""
from __future__ import annotations

from PyQt5 import QtGui

T = {
    # surfaces
    "app": "#eceef1", "surface": "#f8f9fa", "raised": "#ffffff", "selected": "#dfe8f7", "sunken": "#e2e5e9",
    "chrome": "#20252c", "chrome_raised": "#2c323b", "chrome_line": "#3a414c",
    # lines
    "line": "#d7dbe0", "line_strong": "#bcc2ca",
    # text
    "text": "#1b2026", "text2": "#515a65", "muted": "#808a96", "disabled": "#aeb5bd",
    "on_chrome": "#eef0f3", "on_chrome2": "#9ba4b0", "on_chrome3": "#6d7682",
    # semantic
    "accent": "#2b5fb8", "accent_hover": "#244f9c", "accent_disabled": "#a9bcdc",
    "ok": "#1e8a57", "warn": "#b7700b", "warn_bg": "#f8eeda", "danger": "#c23b32", "danger_hover": "#a5312a",
    "danger_bg": "#f8e6e4", "off": "#a2aab3",
}

SPACE = {"xs": 4, "s": 8, "m": 12, "l": 16, "xl": 24}
CONTROL_H = 28
RADIUS = 3
FONT = {"label": 11, "body": 12, "value": 13, "title": 15, "section": 10}


def mono_font(size: int = FONT["value"]) -> QtGui.QFont:
    f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
    f.setPointSizeF(size * 0.78)
    return f


STYLESHEET = f"""
* {{ font-size: {FONT['body']}px; color: {T['text']}; }}
QMainWindow, QDialog, QWidget#page {{ background: {T['app']}; }}
QToolTip {{ background: {T['chrome']}; color: {T['on_chrome']}; border: none; padding: 4px; }}

QFrame[surface="panel"] {{ background: {T['surface']}; border-radius: {RADIUS}px; }}
QFrame[surface="band"] {{ background: {T['raised']}; border-top: 1px solid {T['line']}; }}
QFrame[line="h"] {{ background: {T['line']}; max-height: 1px; min-height: 1px; border: none; }}
QFrame[line="v"] {{ background: {T['line']}; max-width: 1px; min-width: 1px; border: none; }}

QLabel {{ background: transparent; }}
QLabel[kind="section"] {{ font-size: {FONT['section']}px; font-weight: 700; color: {T['muted']}; letter-spacing: 1px; }}
QLabel[kind="title"] {{ font-size: {FONT['title']}px; font-weight: 600; }}
QLabel[kind="secondary"] {{ color: {T['text2']}; }}
QLabel[kind="muted"] {{ color: {T['muted']}; font-size: {FONT['label']}px; }}
QLabel[kind="field"] {{ color: {T['muted']}; font-size: {FONT['label']}px; }}
QLabel[kind="value"] {{ font-size: {FONT['value']}px; }}
QLabel[kind="warn"] {{ color: {T['warn']}; font-size: {FONT['label']}px; font-weight: 600; }}
QLabel[kind="error"] {{ color: {T['danger']}; }}
QLabel[kind="flow"] {{ color: {T['text2']}; font-size: {FONT['label']}px; }}
QLabel[kind="badge"] {{ background: {T['sunken']}; color: {T['text2']}; font-size: {FONT['section']}px; font-weight: 700;
                        border-radius: 2px; padding: 1px 5px; }}
QLabel[kind="badge_warn"] {{ background: {T['warn_bg']}; color: {T['warn']}; font-size: {FONT['section']}px; font-weight: 700;
                             border-radius: 2px; padding: 1px 5px; }}
QLabel[kind="banner_warn"] {{ background: {T['warn_bg']}; color: {T['warn']}; padding: 6px 8px; border-radius: {RADIUS}px; }}
QLabel[kind="banner_bad"] {{ background: {T['danger_bg']}; color: {T['danger']}; padding: 6px 8px; border-radius: {RADIUS}px; font-weight: 600; }}

QPushButton, QToolButton[btn="true"] {{ min-height: {CONTROL_H - 2}px; padding: 0 10px; border-radius: {RADIUS}px;
    border: 1px solid {T['line_strong']}; background: {T['raised']}; color: {T['text']}; }}
QPushButton:hover {{ background: #f0f2f4; }}
QPushButton:pressed {{ background: {T['sunken']}; }}
QPushButton:disabled {{ color: {T['disabled']}; border-color: {T['line']}; background: transparent; }}
QPushButton::menu-indicator {{ width: 0; }}
QPushButton[role="primary"] {{ background: {T['accent']}; color: white; border: none; font-weight: 600; }}
QPushButton[role="primary"]:hover {{ background: {T['accent_hover']}; }}
QPushButton[role="primary"]:disabled {{ background: {T['accent_disabled']}; color: #f2f5fa; }}
QPushButton[role="danger"] {{ color: {T['danger']}; border-color: {T['danger']}; background: {T['raised']}; font-weight: 600; }}
QPushButton[role="danger"]:hover {{ background: {T['danger_bg']}; }}
QPushButton[role="danger"]:disabled {{ color: {T['disabled']}; border-color: {T['line']}; background: transparent; }}
QPushButton[role="stop"] {{ background: {T['danger']}; color: white; border: none; font-weight: 700; padding: 0 16px; }}
QPushButton[role="stop"]:hover {{ background: {T['danger_hover']}; }}
QPushButton[role="experimental"] {{ color: {T['warn']}; border: 1px solid {T['warn']}; background: {T['warn_bg']}; font-weight: 600; }}
QPushButton[role="experimental"]:disabled {{ color: {T['disabled']}; border-color: {T['line']}; background: transparent; }}
QPushButton[role="quiet"] {{ border: none; background: transparent; color: {T['text2']}; padding: 0 6px; }}
QPushButton[role="quiet"]:hover {{ background: {T['sunken']}; }}
QPushButton[role="quiet"]:disabled {{ color: {T['disabled']}; background: transparent; }}
QPushButton:checked {{ background: {T['selected']}; border-color: {T['accent']}; }}

QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {{ min-height: {CONTROL_H - 4}px; padding: 0 5px; border: 1px solid {T['line']};
    border-radius: {RADIUS}px; background: {T['raised']}; selection-background-color: {T['accent']}; selection-color: white; }}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {{ border-color: {T['accent']}; }}
QDoubleSpinBox:disabled, QSpinBox:disabled, QLineEdit:disabled, QComboBox:disabled {{ color: {T['disabled']}; background: {T['surface']}; }}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button, QSpinBox::up-button, QSpinBox::down-button {{ width: 0; border: none; }}
QDoubleSpinBox[compact="true"], QSpinBox[compact="true"] {{ background: transparent; border-color: transparent; border-bottom-color: {T['line']}; border-radius: 0; }}
QDoubleSpinBox[compact="true"]:focus, QSpinBox[compact="true"]:focus {{ background: {T['raised']}; border-bottom-color: {T['accent']}; }}
QComboBox::drop-down {{ border: none; width: 14px; }}
QComboBox QAbstractItemView {{ border: 1px solid {T['line_strong']}; background: {T['raised']}; selection-background-color: {T['selected']}; selection-color: {T['text']}; outline: 0; }}
QCheckBox, QRadioButton {{ spacing: 5px; background: transparent; }}
QCheckBox:disabled, QRadioButton:disabled {{ color: {T['disabled']}; }}
QCheckBox::indicator, QRadioButton::indicator {{ width: 11px; height: 11px; border: 1px solid {T['line_strong']}; background: {T['raised']}; }}
QCheckBox::indicator {{ border-radius: 2px; }}
QRadioButton::indicator {{ border-radius: 6px; }}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {{ background: {T['accent']}; border-color: {T['accent']}; }}
QCheckBox::indicator:indeterminate {{ background: {T['line_strong']}; }}
QCheckBox::indicator:disabled, QRadioButton::indicator:disabled {{ background: {T['surface']}; border-color: {T['line']}; }}
QMenu {{ background: {T['raised']}; border: 1px solid {T['line_strong']}; padding: 4px 0; }}
QMenu::item {{ padding: 5px 18px; }}
QMenu::item:selected {{ background: {T['selected']}; }}
QMenu::separator {{ height: 1px; background: {T['line']}; margin: 4px 0; }}

QTableWidget {{ background: {T['raised']}; border: none; gridline-color: transparent; alternate-background-color: {T['raised']};
    selection-background-color: {T['selected']}; selection-color: {T['text']}; outline: 0; }}
QTableWidget::item {{ padding: 4px 6px; border-bottom: 1px solid {T['app']}; }}
QHeaderView {{ background: transparent; }}
QHeaderView::section {{ text-align: left; background: transparent; border: none; border-bottom: 1px solid {T['line']}; padding: 4px 6px;
    color: {T['muted']}; font-size: {FONT['section']}px; font-weight: 700; }}
QListWidget {{ background: transparent; border: none; outline: 0; }}
QListWidget::item {{ padding: 7px 8px; border-radius: {RADIUS}px; }}
QListWidget::item:selected {{ background: {T['selected']}; color: {T['text']}; }}
QListWidget::item:hover:!selected {{ background: {T['sunken']}; }}
QListWidget#nav {{ background: {T['chrome']}; font-weight: 600; font-size: {FONT['label']}px; }}
QListWidget#nav::item {{ color: {T['on_chrome2']}; padding: 11px 12px; border-left: 2px solid transparent; border-radius: 0; }}
QListWidget#nav::item:selected {{ background: {T['chrome_raised']}; color: {T['on_chrome']}; border-left: 2px solid #7ea6e8; }}
QListWidget#nav::item:hover:!selected {{ background: {T['chrome_raised']}; }}

QPlainTextEdit {{ background: {T['raised']}; border: none; }}
QProgressBar {{ background: {T['sunken']}; border: none; border-radius: 2px; max-height: 6px; min-height: 6px; text-align: right; color: transparent; }}
QProgressBar::chunk {{ background: {T['accent']}; border-radius: 2px; }}
QProgressBar[tone="neutral"]::chunk {{ background: {T['text2']}; }}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QStackedWidget, QTabWidget > QWidget {{ background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 8px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {T['line_strong']}; border-radius: 3px; min-height: 24px; }}
QScrollBar:horizontal {{ background: transparent; height: 8px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: {T['line_strong']}; border-radius: 3px; min-width: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line, QScrollBar::add-page, QScrollBar::sub-page {{ background: none; width: 0; height: 0; }}
QTabWidget::pane {{ border: none; border-top: 1px solid {T['line']}; }}
QTabBar::tab {{ background: transparent; color: {T['text2']}; padding: 5px 12px; border: none; border-bottom: 2px solid transparent; }}
QTabBar::tab:selected {{ color: {T['text']}; font-weight: 600; border-bottom: 2px solid {T['accent']}; }}
QSplitter::handle {{ background: transparent; width: {SPACE['s']}px; }}
QFrame#header {{ background: {T['chrome']}; }}
QFrame#header QLabel {{ color: {T['on_chrome']}; }}
QFrame#header QLabel[kind="field"] {{ color: {T['on_chrome3']}; font-size: {FONT['section']}px; font-weight: 700; letter-spacing: 1px; }}
QFrame#header QLabel[kind="meta"] {{ color: {T['on_chrome3']}; font-size: {FONT['section']}px; }}
QFrame#header QLabel[kind="led"] {{ color: {T['on_chrome2']}; font-size: {FONT['section']}px; }}
QFrame#header QLabel[kind="robot"] {{ font-size: {FONT['title']}px; font-weight: 700; }}
QFrame#header QLabel[kind="state"] {{ color: {T['on_chrome2']}; font-size: {FONT['label']}px; font-weight: 600; }}
QFrame#header QLabel[kind="hvalue"] {{ font-size: {FONT['value']}px; font-weight: 600; }}
QFrame#header QFrame[line="v"] {{ background: {T['chrome_line']}; }}
QFrame#header QComboBox {{ background: transparent; color: {T['on_chrome']}; border: none; border-bottom: 1px solid {T['chrome_line']};
    border-radius: 0; font-size: {FONT['value']}px; font-weight: 600; padding: 0; min-height: 20px; }}
QFrame#header QComboBox QAbstractItemView {{ color: {T['text']}; }}
QFrame#logbar {{ background: {T['sunken']}; }}
QToolButton {{ border: none; background: transparent; color: {T['text2']}; font-weight: 600; font-size: {FONT['label']}px; }}
"""

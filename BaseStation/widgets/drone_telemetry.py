import math
import tkinter as tk
from collections import deque


class DroneTelemetry:
    """Responsive pitch/roll dashboard. Call update() on the Tk main thread."""

    BG = '#10151d'
    CARD = '#1b2430'
    GRID = '#344252'
    TEXT = '#e8f0f8'
    MUTED = '#9aabbc'
    PITCH = '#5fd6ff'
    ROLL = '#ffb74d'

    def __init__(self, root):
        self.root = root
        self.max_points = 60
        self.pitch_history = deque(maxlen=self.max_points)
        self.roll_history = deque(maxlen=self.max_points)
        self.pitch = 0.0
        self.roll = 0.0
        self._redraw_pending = False

        root.configure(bg=self.BG)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)
        root.rowconfigure(2, weight=1)

        tk.Label(root, text='FLIGHT ATTITUDE', font=('DejaVu Sans', 13, 'bold'),
                 fg=self.TEXT, bg=self.BG, anchor='w').grid(
                     row=0, column=0, columnspan=2, sticky='ew', padx=14, pady=(12, 5))

        self.canvases = {}
        self.readouts = {}
        for column, (name, color) in enumerate((('Pitch', self.PITCH), ('Roll', self.ROLL))):
            card = tk.Frame(root, bg=self.CARD, highlightbackground=self.GRID,
                            highlightthickness=1)
            card.grid(row=1, column=column, sticky='nsew', padx=(12 if column == 0 else 5,
                                                                12 if column == 1 else 5), pady=6)
            card.columnconfigure(0, weight=1)
            card.rowconfigure(2, weight=1)
            tk.Label(card, text=name.upper(), fg=self.MUTED, bg=self.CARD,
                     font=('DejaVu Sans', 10, 'bold')).grid(row=0, column=0, pady=(10, 0))
            value = tk.Label(card, text='+0.0°', fg=color, bg=self.CARD,
                             font=('DejaVu Sans', 23, 'bold'))
            value.grid(row=1, column=0, pady=(0, 2))
            self.readouts[name] = value
            canvas = tk.Canvas(card, width=190, height=145, bg=self.CARD,
                               highlightthickness=0)
            canvas.grid(row=2, column=0, sticky='nsew', padx=8, pady=(0, 8))
            canvas.bind('<Configure>', self._schedule_redraw)
            self.canvases[name] = canvas

        for column, (name, color) in enumerate((('Pitch', self.PITCH), ('Roll', self.ROLL))):
            card = tk.Frame(root, bg=self.CARD, highlightbackground=self.GRID,
                            highlightthickness=1)
            card.grid(row=2, column=column, sticky='nsew', padx=(12 if column == 0 else 5,
                                                                12 if column == 1 else 5), pady=(5, 12))
            card.columnconfigure(0, weight=1)
            card.rowconfigure(1, weight=1)
            tk.Label(card, text=f'{name.upper()} HISTORY  ·  ±90°', fg=self.MUTED,
                     bg=self.CARD, font=('DejaVu Sans', 9, 'bold'),
                     anchor='w').grid(row=0, column=0, sticky='ew', padx=12, pady=(10, 0))
            canvas = tk.Canvas(card, width=190, height=125, bg=self.CARD,
                               highlightthickness=0)
            canvas.grid(row=1, column=0, sticky='nsew', padx=6, pady=(2, 8))
            canvas.bind('<Configure>', self._schedule_redraw)
            self.canvases[name + 'History'] = canvas

        self._redraw()

    def _schedule_redraw(self, _event=None):
        if not self._redraw_pending:
            self._redraw_pending = True
            self.root.after_idle(self._redraw)

    def _redraw(self):
        self._redraw_pending = False
        self._draw_attitude(self.canvases['Pitch'], self.pitch, self.PITCH, 'pitch')
        self._draw_attitude(self.canvases['Roll'], self.roll, self.ROLL, 'roll')
        self._draw_history(self.canvases['PitchHistory'], self.pitch_history, self.PITCH)
        self._draw_history(self.canvases['RollHistory'], self.roll_history, self.ROLL)

    def _draw_attitude(self, canvas, angle, color, axis):
        canvas.delete('all')
        w, h = canvas.winfo_width(), canvas.winfo_height()
        if w < 30 or h < 30:
            return
        cx, cy = w / 2, h / 2
        radius = max(12, min(w * .37, h * .39))
        canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius,
                           outline=self.GRID, width=2)
        canvas.create_line(cx-radius-7, cy, cx+radius+7, cy, fill=self.GRID, dash=(3, 4))
        canvas.create_line(cx, cy-radius-7, cx, cy+radius+7, fill=self.GRID, dash=(3, 4))
        radians = math.radians(angle)
        if axis == 'roll':
            # Rotating wings: positive roll lowers the right wing on screen.
            dx = math.cos(radians) * radius * .85
            dy = math.sin(radians) * radius * .85
            canvas.create_line(cx-dx, cy-dy, cx+dx, cy+dy,
                               fill=color, width=5, capstyle=tk.ROUND)
            canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill=self.TEXT, outline='')
            canvas.create_text(cx, h-7, text='LEFT  ←     →  RIGHT', fill=self.MUTED,
                               font=('DejaVu Sans', 8))
        else:
            # Side-view nose marker: positive pitch moves the nose upward.
            offset = math.sin(radians) * radius * .75
            canvas.create_line(cx-radius*.72, cy, cx+radius*.72, cy,
                               fill=self.GRID, width=2)
            canvas.create_line(cx-radius*.55, cy, cx+radius*.55, cy-offset,
                               fill=color, width=5, capstyle=tk.ROUND)
            canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill=self.TEXT, outline='')
            canvas.create_text(cx, h-7, text='NOSE DOWN  /  NOSE UP', fill=self.MUTED,
                               font=('DejaVu Sans', 8))

    def _draw_history(self, canvas, history, color):
        canvas.delete('all')
        w, h = canvas.winfo_width(), canvas.winfo_height()
        if w < 60 or h < 50:
            return
        left, right, top, bottom = 33, w-10, 9, h-14
        if right <= left or bottom <= top:
            return
        for value in (-90, -45, 0, 45, 90):
            y = bottom - (value + 90) / 180 * (bottom-top)
            canvas.create_line(left, y, right, y, fill=self.GRID,
                               dash=(3, 3) if value else (), width=1)
            canvas.create_text(left-5, y, text=f'{value:+d}°' if value else '0°',
                               fill=self.MUTED, anchor='e', font=('DejaVu Sans', 8))
        if not history:
            return
        points = []
        # Fixed-width time axis: new measurements enter at the right edge.
        start = self.max_points - len(history)
        for index, value in enumerate(history):
            x = left + (start + index) / (self.max_points - 1) * (right-left)
            clipped = max(-90, min(90, value))
            y = bottom - (clipped + 90) / 180 * (bottom-top)
            points.extend((x, y))
        if len(points) >= 4:
            canvas.create_line(*points, fill=color, width=2, smooth=False)
        x, y = points[-2:]
        canvas.create_oval(x-3, y-3, x+3, y+3, fill=color, outline='')

    def update(self, roll, pitch):
        """Must be called from the Tkinter main thread."""
        self.roll = float(roll)
        self.pitch = float(pitch)
        if not (math.isfinite(self.roll) and math.isfinite(self.pitch)):
            return
        self.roll_history.append(self.roll)
        self.pitch_history.append(self.pitch)
        self.readouts['Pitch'].configure(text=f'{self.pitch:+.1f}°')
        self.readouts['Roll'].configure(text=f'{self.roll:+.1f}°')
        self._schedule_redraw()

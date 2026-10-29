import tkinter as tk


class Throttles:
    BG = '#10151d'
    CARD = '#1b2430'
    BORDER = '#344252'
    TEXT = '#e8f0f8'
    MUTED = '#9aabbc'
    CYAN = '#5fd6ff'
    ORANGE = '#ffb74d'

    def __init__(self, root, onHeightThrottleChange, onYawThrottleChange,
                 onPitchThrottleChange, onRollThrottleChange):
        self.root = root
        self.callbacks = dict(height=onHeightThrottleChange, yaw=onYawThrottleChange,
                              pitch=onPitchThrottleChange, roll=onRollThrottleChange)
        self.values = {name: tk.DoubleVar(value=0) for name in self.callbacks}
        self.height_throttle = self.values['height']
        self.yaw_throttle = self.values['yaw']
        self.pitch_throttle = self.values['pitch']
        self.roll_throttle = self.values['roll']
        self.readouts = {}
        self.scales = {}
        root.configure(bg=self.BG)
        root.columnconfigure((0, 1), weight=1, uniform='controls')
        root.rowconfigure(1, weight=1)
        root.rowconfigure(2, weight=1)
        tk.Label(root, text='FLIGHT CONTROLS', bg=self.BG, fg=self.TEXT,
                 font=('DejaVu Sans', 13, 'bold'), anchor='w').grid(
                     row=0, column=0, columnspan=2, sticky='ew', padx=14, pady=(12, 5))
        self._make_control('height', 'THROTTLE', 0, 1, True, self.CYAN)
        self._make_control('pitch', 'PITCH', 1, 1, True, self.ORANGE)
        self._make_control('yaw', 'YAW', 0, 2, False, self.CYAN)
        self._make_control('roll', 'ROLL', 1, 2, False, self.ORANGE)
        footer = tk.Frame(root, bg=self.BG)
        footer.grid(row=3, column=0, columnspan=2, sticky='ew', padx=12, pady=(4, 12))
        tk.Button(footer, text='CENTER ATTITUDE / YAW', command=self.center_sticks,
                  bg=self.BORDER, fg=self.TEXT, activebackground=self.CARD,
                  activeforeground=self.TEXT, relief='flat', bd=0,
                  font=('DejaVu Sans', 9, 'bold'), padx=10, pady=9,
                  cursor='hand2').pack(fill='x')
        tk.Label(footer, text='Throttle is not changed by center button',
                 bg=self.BG, fg=self.MUTED, font=('DejaVu Sans', 8)).pack(pady=(6, 0))

    def _make_control(self, name, title, column, row, vertical, color):
        card = tk.Frame(self.root, bg=self.CARD, highlightbackground=self.BORDER,
                        highlightthickness=1)
        card.grid(row=row, column=column, sticky='nsew', padx=(12 if column == 0 else 5,
                   12 if column == 1 else 5), pady=6)
        card.columnconfigure(0, weight=1)
        card.rowconfigure(2, weight=1)
        tk.Label(card, text=title, bg=self.CARD, fg=self.MUTED,
                 font=('DejaVu Sans', 9, 'bold')).grid(row=0, column=0, pady=(10, 0))
        label = tk.Label(card, text='+0%', bg=self.CARD, fg=color,
                         font=('DejaVu Sans', 21, 'bold'))
        label.grid(row=1, column=0, pady=(0, 2))
        self.readouts[name] = label
        scale = tk.Scale(card, from_=100 if vertical else -100,
                         to=-100 if vertical else 100,
                         orient=tk.VERTICAL if vertical else tk.HORIZONTAL,
                         variable=self.values[name], resolution=1, showvalue=False,
                         command=lambda value, key=name: self._changed(key, value),
                         length=175 if vertical else 190, width=15, sliderlength=22,
                         bg=self.CARD, fg=self.TEXT, troughcolor=self.BG,
                         activebackground=color, highlightthickness=0, bd=0,
                         relief='flat', font=('DejaVu Sans', 8))
        scale.grid(row=2, column=0, sticky='nsew' if vertical else 'ew', padx=14, pady=5)
        self.scales[name] = scale
        tk.Label(card, text=('−100      0      +100' if not vertical else '+100  /  0  /  −100'),
                 bg=self.CARD, fg=self.MUTED,
                 font=('DejaVu Sans', 8)).grid(row=3, column=0, pady=(0, 10))

    def _changed(self, name, value):
        value = max(-100.0, min(100.0, float(value)))
        self.readouts[name].configure(text=f'{value:+.0f}%')
        self.callbacks[name](value)

    def set_control(self, name, value):
        """Call only on the Tk main thread. Used for controller events."""
        if name not in self.scales:
            raise ValueError(f'Unknown control: {name}')
        value = max(-100.0, min(100.0, float(value)))
        if abs(self.values[name].get() - value) >= 0.5:
            self.scales[name].set(round(value))

    def center_sticks(self):
        for name in ('yaw', 'pitch', 'roll'):
            self.set_control(name, 0)

    # Backward-compatible methods. Must be invoked on the Tk main thread.
    def on_height_throttle_change(self, val): self.set_control('height', val)
    def on_yaw_throttle_change(self, val): self.set_control('yaw', val)
    def on_pitch_throttle_change(self, val): self.set_control('pitch', val)
    def on_roll_throttle_change(self, val): self.set_control('roll', val)

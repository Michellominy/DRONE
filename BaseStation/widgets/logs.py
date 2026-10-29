import tkinter as tk
from datetime import datetime


class Logs:
    BG = '#10151d'
    CARD = '#1b2430'
    BORDER = '#344252'
    TEXT = '#e8f0f8'
    MUTED = '#9aabbc'

    def __init__(self, root):
        root.configure(bg=self.BG)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)
        header = tk.Frame(root, bg=self.BG)
        header.grid(row=0, column=0, sticky='ew', padx=14, pady=(10, 6))
        tk.Label(header, text='SYSTEM CONSOLE', bg=self.BG, fg=self.TEXT,
                 font=('DejaVu Sans', 11, 'bold')).pack(side='left')
        self.counter = tk.Label(header, text='0 messages', bg=self.BG, fg=self.MUTED,
                                font=('DejaVu Sans', 9))
        self.counter.pack(side='left', padx=15)
        tk.Button(header, text='CLEAR LOG', command=self.clear, bg=self.CARD,
                  fg=self.TEXT, activebackground=self.BORDER, activeforeground=self.TEXT,
                  relief='flat', bd=0, padx=12, pady=5, cursor='hand2').pack(side='right')
        body = tk.Frame(root, bg=self.CARD, highlightbackground=self.BORDER, highlightthickness=1)
        body.grid(row=1, column=0, sticky='nsew', padx=12, pady=(0, 12))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        self.log_text = tk.Text(body, wrap='word', bg=self.CARD, fg=self.TEXT,
                                insertbackground=self.TEXT, selectbackground=self.BORDER,
                                font=('DejaVu Sans Mono', 9), relief='flat', bd=0,
                                padx=12, pady=10, state='disabled', height=8)
        self.log_text.grid(row=0, column=0, sticky='nsew')
        scroll = tk.Scrollbar(body, command=self.log_text.yview, bg=self.CARD,
                              troughcolor=self.BG, activebackground=self.BORDER)
        scroll.grid(row=0, column=1, sticky='ns')
        self.log_text.configure(yscrollcommand=scroll.set)
        for tag, color in [('time', self.MUTED), ('info', '#5fd6ff'),
                           ('warn', '#ffb74d'), ('error', '#ff7373')]:
            self.log_text.tag_configure(tag, foreground=color)
        self.count = 0

    def update_log(self, message):
        message = str(message).rstrip('\r\n')
        if not message:
            return
        level = ('error' if any(x in message.upper() for x in ('ERROR', 'FAIL', 'CRITICAL'))
                 else 'warn' if any(x in message.upper() for x in ('WARN', 'CAUTION')) else 'info')
        near_bottom = self.log_text.yview()[1] >= 0.98
        self.log_text.configure(state='normal')
        for line in message.splitlines():
            self.log_text.insert('end', datetime.now().strftime('%H:%M:%S') + '  ', 'time')
            self.log_text.insert('end', line + '\n', level)
            self.count += 1
        # Keep the console bounded for long-running sessions.
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 2000:
            self.log_text.delete('1.0', f'{lines - 1500}.0')
        self.log_text.configure(state='disabled')
        self.counter.configure(text=f'{self.count} messages')
        if near_bottom:
            self.log_text.see('end')

    def clear(self):
        self.log_text.configure(state='normal')
        self.log_text.delete('1.0', 'end')
        self.log_text.configure(state='disabled')
        self.count = 0
        self.counter.configure(text='0 messages')

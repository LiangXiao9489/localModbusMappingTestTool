import tkinter as tk
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading, queue, time, struct, sys, json, os, random

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BG = "#0b1220"
PANEL = "#111c2e"
PANEL2 = "#16243a"
FG = "#dce8f7"
MUTED = "#8193ad"
CYAN = "#38bdf8"
GREEN = "#34d399"
RED = "#fb7185"
LINE = "#243650"


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)


class Device:
    def __init__(self):
        self.lock = threading.RLock()
        self.items = []
        self.running = False
        self.ser = None
        self.thread = None
        self.io_count = 0

    def snapshot(self):
        with self.lock:
            return [dict(x) for x in self.items]

    def set_items(self, items):
        with self.lock:
            self.items = items

    def reset_values(self):
        with self.lock:
            for item in self.items:
                item["value"] = (
                    list(item.get("default", 0))
                    if isinstance(item.get("default", 0), list)
                    else item.get("default", 0)
                )

    def find(self, kind, address):
        return next(
            (x for x in self.items if x["kind"] == kind and x["address"] == address),
            None,
        )

    def value_at(self, kind, address):
        for item in self.items:
            if item["kind"] == kind and item["address"] <= address < item[
                "address"
            ] + int(item.get("length", 1)):
                value = item.get("value", 0)
                return (
                    value[address - item["address"]]
                    if isinstance(value, list)
                    else value
                )
        return 0

    def set_value_at(self, kind, address, value):
        for item in self.items:
            if item["kind"] == kind and item["address"] <= address < item[
                "address"
            ] + int(item.get("length", 1)):
                offset = address - item["address"]
                if isinstance(item.get("value"), list):
                    item["value"][offset] = value
                else:
                    item["value"] = value
                return

    def matched_items(self, kind, start, qty):
        end = start + qty
        return [
            x
            for x in self.items
            if x["kind"] == kind
            and x["address"] < end
            and x["address"] + int(x.get("length", 1)) > start
        ]

    def read_bits(self, kind, start, qty):
        with self.lock:
            return [int(self.value_at(kind, start + i)) for i in range(qty)]

    def read_regs(self, kind, start, qty):
        with self.lock:
            return [int(self.value_at(kind, start + i)) for i in range(qty)]

    def start(self, port, baud, parity, stop, data, slave, log):
        if serial is None:
            raise RuntimeError(
                "未安装 pyserial，请先执行：python -m pip install -r requirements.txt"
            )
        with self.lock:
            self.io_count = 0
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            parity=parity,
            stopbits=stop,
            bytesize=data,
            timeout=0.15,
        )
        self.running = True
        self.thread = threading.Thread(target=self.loop, args=(slave, log), daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def loop(self, slave, log):
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
                while len(buf) >= 4:
                    if buf[0] != slave:
                        buf = buf[1:]
                        continue
                    # Modbus RTU frame ends by a silent interval; use expected length where possible.
                    fn = buf[1]
                    need = None
                    if fn in (1, 2, 3, 4, 5, 6):
                        need = 8
                    elif fn in (15, 16) and len(buf) >= 7:
                        need = 9 + buf[6]
                    if need is None or len(buf) < need:
                        break
                    frame, buf = buf[:need], buf[need:]
                    if frame[-2:] != crc16(frame[:-2]):
                        log("CRC 错误")
                        continue
                    rsp = self.handle(frame, log)
                    if rsp:
                        self.ser.write(rsp)
            except Exception as e:
                if self.running:
                    log("串口错误: " + str(e))
                    time.sleep(0.3)

    def handle(self, f, log):
        fn = f[1]
        with self.lock:
            self.io_count += 1
            count = self.io_count
        try:
            if fn in (1, 2, 3, 4):
                start, qty = struct.unpack(">HH", f[2:6])
                kind = {1: "线圈", 2: "离散输入", 3: "保持寄存器", 4: "输入寄存器"}[fn]
                if not 1 <= qty <= (2000 if fn < 3 else 125):
                    raise ValueError()
                if fn < 3:
                    vals = self.read_bits(kind, start, qty)
                    out = bytearray((qty + 7) // 8)
                    for i, v in enumerate(vals):
                        out[i // 8] |= (v & 1) << (i % 8)
                else:
                    out = bytearray().join(
                        struct.pack(">H", v & 0xFFFF)
                        for v in self.read_regs(kind, start, qty)
                    )
                matched = self.matched_items(kind, start, qty)
                if matched:
                    for item in matched:
                        log(
                            f'[{count}]被网关 读 {kind} {item["address"]}，寄存器长度[ {item.get("length",1)} ]'
                        )
                else:
                    log(f"[{count}]被网关 读 {kind} {start}，寄存器长度[ {qty} ]")
                return (
                    bytes([f[0], fn, len(out)])
                    + bytes(out)
                    + crc16(bytes([f[0], fn, len(out)]) + bytes(out))
                )
            if fn in (5, 6):
                addr, val = struct.unpack(">HH", f[2:6])
                kind = "线圈" if fn == 5 else "保持寄存器"
                item = self.find(kind, addr)
                if fn == 5 and val not in (0x0000, 0xFF00):
                    raise ValueError()
                self.set_value_at(
                    kind, addr, (1 if val == 0xFF00 else 0) if fn == 5 else val
                )
                matched = self.matched_items(kind, addr, 1)
                if matched:
                    for item in matched:
                        log(
                            f'[{count}]被网关 写 {kind} {item["address"]}，寄存器长度[ {item.get("length",1)} ]'
                        )
                else:
                    log(f"[{count}]被网关 写 {kind} {addr}，寄存器长度[ 1 ]")
                return f
            if fn in (15, 16):
                addr, qty = struct.unpack(">HH", f[2:6])
                byte_count = f[6]
                payload = f[7 : 7 + byte_count]
                if fn == 15:
                    if (
                        not 1 <= qty <= 1968
                        or byte_count != (qty + 7) // 8
                        or len(payload) != byte_count
                    ):
                        raise ValueError()
                    for i in range(qty):
                        self.set_value_at(
                            "线圈", addr + i, (payload[i // 8] >> (i % 8)) & 1
                        )
                else:
                    if (
                        not 1 <= qty <= 123
                        or byte_count != qty * 2
                        or len(payload) != byte_count
                    ):
                        raise ValueError()
                    for i in range(qty):
                        self.set_value_at(
                            "保持寄存器",
                            addr + i,
                            struct.unpack(">H", payload[i * 2 : i * 2 + 2])[0],
                        )
                matched = self.matched_items(
                    "线圈" if fn == 15 else "保持寄存器", addr, qty
                )
                if matched:
                    for item in matched:
                        log(
                            f'[{count}]被网关 写 {item["kind"]} {item["address"]}，寄存器长度[ {item.get("length",1)} ]'
                        )
                else:
                    log(f"[{count}]被网关 写 {addr}，寄存器长度[ {qty} ]")
                out = f[:6]
                return out + crc16(out)
        except Exception:
            pass
        err = bytes([f[0], fn | 0x80, 2])
        return err + crc16(err)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("485 从机模拟器")
        self.geometry("1520x720")
        self.minsize(950, 600)
        self.configure(bg=BG)
        self.dev = Device()
        self.events = queue.Queue()
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.base_dir, "registers.json")
        self.tables_dir = os.path.join(self.base_dir, "point_tables")
        self.current_table_path = None
        self.current_table_name = "未选择点表"
        self.prepare_tables()
        self.configure_style()
        self.build()
        self.load_config()
        self.after(200, self.refresh)
        self.protocol("WM_DELETE_WINDOW", self.close)

    def configure_style(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(
            ".",
            background=BG,
            foreground=FG,
            fieldbackground=PANEL2,
            font=("Segoe UI", 10),
        )
        s.configure("TFrame", background=BG)
        s.configure("Card.TFrame", background=PANEL)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("Muted.TLabel", background=BG, foreground=MUTED)
        s.configure(
            "Title.TLabel", background=BG, foreground=FG, font=("Segoe UI Semibold", 20)
        )
        s.configure(
            "TButton",
            background=PANEL2,
            foreground=FG,
            padding=(12, 5),
            anchor="center",
        )
        s.map("TButton", background=[("active", "#203855")])
        s.configure(
            "Accent.TButton",
            background="#0ea5e9",
            foreground="white",
            padding=(12, 5),
            anchor="center",
        )
        s.configure(
            "TCombobox",
            foreground=FG,
            fieldforeground=FG,
            background=PANEL2,
            arrowcolor=CYAN,
        )
        s.map(
            "TCombobox",
            fieldbackground=[("readonly", PANEL2)],
            foreground=[("readonly", FG)],
        )
        s.configure(
            "Treeview",
            background=PANEL,
            fieldbackground=PANEL,
            foreground=FG,
            rowheight=30,
        )
        s.configure("Treeview.Heading", background=PANEL2, foreground=CYAN)

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=24, pady=(20, 10))
        ttk.Label(top, text="485 从机模拟器", style="Title.TLabel").pack(side="left")
        self.current_label = ttk.Label(
            top, text="当前点表：未选择点表", foreground=CYAN
        )
        self.current_label.pack(side="right", padx=24, pady=8)
        self.status = ttk.Label(top, text="● 已停止", foreground=RED)
        self.status.pack(side="right", pady=8)
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=24, pady=6)
        self.vars = {
            k: tk.StringVar(value=v)
            for k, v in {
                "port": "COM1",
                "baud": "9600",
                "parity": "无",
                "stop": "1",
                "data": "8",
                "slave": "1",
            }.items()
        }
        for label, key, values in [
            ("串口", "port", None),
            (
                "波特率",
                "baud",
                ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"],
            ),
            ("校验", "parity", ["无", "奇", "偶"]),
            ("停止位", "stop", ["1", "2"]),
            ("数据位", "data", ["7", "8"]),
            ("从机地址", "slave", None),
        ]:
            box = ttk.Frame(bar)
            box.pack(side="left", padx=(0, 16))
            ttk.Label(box, text=label, style="Muted.TLabel").pack(anchor="w")
            w = (
                ttk.Entry(box, textvariable=self.vars[key], width=10)
                if key == "slave"
                else ttk.Combobox(
                    box,
                    textvariable=self.vars[key],
                    values=values or [],
                    width=10,
                    state="normal" if not values else "readonly",
                )
            )
            w.pack(pady=(4, 0))
            if key == "port":
                self.portbox = w
        ttk.Button(bar, text="刷新串口", command=self.ports).pack(
            side="left", padx=4, pady=18
        )
        self.resetbtn = ttk.Button(
            bar, text="重置寄存器值", command=self.reset_registers
        )
        self.resetbtn.pack(side="right", padx=(8, 0), pady=18)
        self.loopbtn = ttk.Button(
            bar, text="本地回环测试", command=self.loop_test, state="disabled"
        )
        self.loopbtn.pack(side="right", padx=(8, 0), pady=18)
        self.tablebtn = ttk.Button(bar, text="选择从机点表", command=self.select_table)
        self.tablebtn.pack(side="right", pady=18)
        self.configbtn = self.tablebtn
        self.startbtn = ttk.Button(
            bar, text="▶  启动模拟", style="Accent.TButton", command=self.toggle
        )
        self.startbtn.pack(side="right", padx=8, pady=18)
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=24, pady=10)
        left = ttk.Frame(body, style="Card.TFrame")
        left.pack(side="left", fill="both", expand=True)
        ttk.Label(
            left,
            text="寄存器总览",
            font=("Segoe UI Semibold", 13),
            background=PANEL,
            foreground=FG,
        ).pack(anchor="w", padx=16, pady=14)
        cols = (
            "number",
            "label",
            "type",
            "addr",
            "mapping",
            "length",
            "value",
            "default",
        )
        self.tree = ttk.Treeview(left, columns=cols, show="headings")
        [
            self.tree.heading(c, text=t)
            for c, t in zip(
                cols,
                [
                    "编号",
                    "标签",
                    "类型",
                    "地址",
                    "映射地址",
                    "寄存器长度",
                    "当前值",
                    "默认值",
                ],
            )
        ]
        [
            self.tree.column(c, width=w, anchor="center")
            for c, w in zip(cols, [70, 150, 110, 85, 100, 100, 110, 110])
        ]
        self.tree.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        right = ttk.Frame(body, style="Card.TFrame", width=500)
        right.pack(side="right", fill="y", padx=(16, 0))
        right.pack_propagate(False)
        ttk.Label(
            right,
            text="运行日志",
            font=("Segoe UI Semibold", 13),
            background=PANEL,
            foreground=FG,
        ).pack(anchor="w", padx=16, pady=14)
        self.logbox = tk.Text(
            right,
            bg=PANEL,
            fg=MUTED,
            insertbackground=FG,
            relief="flat",
            state="disabled",
            wrap="word",
        )
        self.logbox.pack(fill="both", expand=True, padx=12, pady=(0, 12))

    def ports(self):
        if list_ports:
            self.portbox["values"] = [p.device for p in list_ports.comports()]

    def edit(self, initial_items=None, on_saved=None, table_path=None):
        table_path_holder = table_path if isinstance(table_path, list) else [table_path]
        win = tk.Toplevel(self)
        win.title("配置寄存器列表")
        win.geometry("1000x800")
        win.configure(bg=BG)
        win.transient(self)
        win.grab_set()
        frame = ttk.Frame(win)
        frame.pack(fill="both", expand=True, padx=18, pady=18)
        ttk.Label(
            frame, text="寄存器列表（双击一行可编辑）", style="Title.TLabel"
        ).pack(anchor="w", pady=(0, 12))
        cols = ("number", "label", "type", "address", "mapping", "length", "default")
        tree = ttk.Treeview(frame, columns=cols, show="headings", height=15)
        [
            tree.heading(c, text=t)
            for c, t in zip(
                cols,
                ["编号", "标签", "类型", "地址", "映射地址", "寄存器长度", "默认值"],
            )
        ]
        [
            tree.column(c, width=w, anchor="center")
            for c, w in zip(cols, [70, 180, 150, 120, 120, 120, 140])
        ]
        tree.pack(fill="both", expand=True)
        items = sorted(
            initial_items if initial_items is not None else self.dev.snapshot(),
            key=lambda z: (z["kind"], z["address"]),
        )
        [
            tree.insert(
                "",
                "end",
                values=(
                    i,
                    x.get("label", ""),
                    x["kind"],
                    x["address"],
                    x.get("mapping", x["address"]),
                    x.get("length", 1),
                    (
                        ",".join(map(str, x["default"]))
                        if isinstance(x.get("default"), list)
                        else x["default"]
                    ),
                ),
            )
            for i, x in enumerate(items, 1)
        ]

        def edit_row(event=None):
            selected = tree.selection()
            if not selected:
                return
            iid = selected[0]
            old = list(tree.item(iid, "values"))
            ew = tk.Toplevel(win)
            ew.title("编辑寄存器")
            ew.geometry("620x230")
            ew.configure(bg=BG)
            ew.transient(win)
            ew.grab_set()
            ef = ttk.Frame(ew)
            ef.pack(fill="both", expand=True, padx=16, pady=16)
            ev = [tk.StringVar(value=str(old[i])) for i in range(1, 7)]
            labels = [
                "标签",
                "类型",
                "地址",
                "映射地址",
                "寄存器长度",
                "默认值（逗号分隔）",
            ]
            widgets = []
            for i, name in enumerate(labels):
                ttk.Label(ef, text=name, style="Muted.TLabel").grid(
                    row=0, column=i, padx=3, sticky="w"
                )
            widgets.append(ttk.Entry(ef, textvariable=ev[0], width=16))
            widgets.append(
                ttk.Combobox(
                    ef,
                    textvariable=ev[1],
                    values=["线圈", "离散输入", "保持寄存器", "输入寄存器"],
                    state="readonly",
                    width=14,
                )
            )
            widgets += [
                ttk.Entry(ef, textvariable=ev[2], width=12),
                ttk.Entry(ef, textvariable=ev[3], width=12),
                ttk.Entry(ef, textvariable=ev[4], width=12),
                ttk.Entry(ef, textvariable=ev[5], width=18),
            ]
            for i, w in enumerate(widgets):
                w.grid(row=1, column=i, padx=3, sticky="w")

            def apply_edit():
                try:
                    text, k, a, m, l, v = (
                        ev[0].get(),
                        ev[1].get(),
                        int(ev[2].get()),
                        int(ev[3].get()),
                        int(ev[4].get()),
                        [int(x.strip()) for x in ev[5].get().split(",")],
                    )
                    assert (
                        len(text) <= 32
                        and text.isascii()
                        and (not text or text.isalnum())
                        and 0 <= a <= 65535
                        and 0 <= m <= 65535
                        and 1 <= l <= 1968
                        and len(v) == l
                        and all(0 <= x <= 65535 for x in v)
                    )
                except:
                    messagebox.showerror(
                        "参数错误",
                        "请检查标签、地址、映射地址、长度和默认值；默认值数量必须等于寄存器长度。",
                        parent=ew,
                    )
                    return
                tree.item(iid, values=(old[0], text, k, a, m, l, ",".join(map(str, v))))
                ew.destroy()

            ttk.Button(
                ef, text="保存修改", style="Accent.TButton", command=apply_edit
            ).grid(row=2, column=5, pady=18, sticky="e")

        tree.bind("<Double-1>", edit_row)
        form = ttk.Frame(frame)
        form.pack(fill="x", pady=12)
        kind = tk.StringVar(value="保持寄存器")
        addr = tk.StringVar(value="0")
        mapping = tk.StringVar(value="0")
        length = tk.StringVar(value="1")
        val = tk.StringVar(value="0")
        label = tk.StringVar(value="")
        ttk.Label(form, text="类型", style="Muted.TLabel").pack(
            side="left", padx=(0, 3)
        )
        ttk.Label(form, text="地址", style="Muted.TLabel").pack(
            side="left", padx=(10, 3)
        )
        ttk.Label(form, text="默认值", style="Muted.TLabel").pack(
            side="left", padx=(10, 3)
        )
        # Labels are placed above the controls in a dedicated row below.
        for w in form.winfo_children():
            w.pack_forget()
        fields = ttk.Frame(frame)
        fields.pack(fill="x", pady=(0, 8))
        ttk.Label(fields, text="类型", style="Muted.TLabel").grid(
            row=0, column=0, padx=3, sticky="w"
        )
        ttk.Label(fields, text="地址（0-65535）", style="Muted.TLabel").grid(
            row=0, column=1, padx=3, sticky="w"
        )
        ttk.Label(fields, text="映射地址（0-65535）", style="Muted.TLabel").grid(
            row=0, column=2, padx=3, sticky="w"
        )
        ttk.Label(fields, text="寄存器长度（默认1）", style="Muted.TLabel").grid(
            row=0, column=3, padx=3, sticky="w"
        )
        ttk.Label(fields, text="默认值（逗号分隔）", style="Muted.TLabel").grid(
            row=0, column=4, padx=3, sticky="w"
        )
        ttk.Label(
            fields, text="寄存器标签（英文和数字，最多32字符）", style="Muted.TLabel"
        ).grid(row=0, column=5, padx=3, sticky="w")
        ttk.Combobox(
            fields,
            textvariable=kind,
            values=["线圈", "离散输入", "保持寄存器", "输入寄存器"],
            state="readonly",
            width=14,
        ).grid(row=1, column=0, padx=3, sticky="w")
        ttk.Entry(fields, textvariable=addr, width=12).grid(
            row=1, column=1, padx=3, sticky="w"
        )
        ttk.Entry(fields, textvariable=mapping, width=14).grid(
            row=1, column=2, padx=3, sticky="w"
        )
        ttk.Entry(fields, textvariable=length, width=14).grid(
            row=1, column=3, padx=3, sticky="w"
        )
        ttk.Entry(fields, textvariable=val, width=14).grid(
            row=1, column=4, padx=3, sticky="w"
        )

        def label_key_ok(proposed):
            return len(proposed) <= 32 and all(
                ("0" <= c <= "9") or ("A" <= c <= "Z") or ("a" <= c <= "z")
                for c in proposed
            )

        label_vcmd = (self.register(label_key_ok), "%P")
        ttk.Entry(
            fields,
            textvariable=label,
            width=20,
            validate="key",
            validatecommand=label_vcmd,
        ).grid(row=1, column=5, padx=3, sticky="w")

        def renumber():
            for number, iid in enumerate(tree.get_children(), 1):
                values = list(tree.item(iid, "values"))
                values[0] = number
                tree.item(iid, values=values)

        for w in form.winfo_children():
            w.destroy()

        def add():
            text = label.get()
            if text and (not text.isalnum() or not text.isascii()):
                messagebox.showerror(
                    "标签输入错误",
                    "寄存器标签只能使用英文和数字，长度不能超过32个字符。",
                    parent=win,
                )
                return
            try:
                a = int(addr.get())
                m = int(mapping.get())
                l = int(length.get())
                raw = [int(x.strip()) for x in val.get().split(",")]
                assert (
                    0 <= a <= 65535
                    and 0 <= m <= 65535
                    and 1 <= l <= 1968
                    and len(raw) == l
                    and all(0 <= x <= 65535 for x in raw)
                )
            except:
                messagebox.showerror(
                    "参数错误",
                    "地址、映射地址、长度和值必须有效；默认值数量必须等于寄存器长度，每个值范围为0-65535。",
                    parent=win,
                )
                return
            tree.insert(
                "",
                "end",
                values=("", text, kind.get(), a, m, l, ",".join(map(str, raw))),
            )
            renumber()

        def delete():
            for i in tree.selection():
                tree.delete(i)
            renumber()

        ttk.Button(form, text="添加", command=add).pack(side="left", padx=6, ipady=2)
        ttk.Button(form, text="删除选中", command=delete).pack(side="left", ipady=2)

        def save():
            out = []
            seen = set()
            for iid in tree.get_children():
                number, text, k, a, m, l, v = tree.item(iid)["values"]
                key = (k, int(a))
                if key in seen:
                    messagebox.showerror(
                        "重复地址", "同一类型下地址不能重复", parent=win
                    )
                    return
                if len(str(text)) > 32 or (
                    str(text) and (not str(text).isalnum() or not str(text).isascii())
                ):
                    messagebox.showerror(
                        "标签输入错误",
                        "寄存器标签只能使用英文和数字，长度不能超过32个字符。",
                        parent=win,
                    )
                    return
                try:
                    defaults = [int(x.strip()) for x in str(v).split(",")]
                    assert len(defaults) == int(l) and all(
                        0 <= x <= 65535 for x in defaults
                    )
                except:
                    messagebox.showerror(
                        "默认值错误",
                        "默认值数量必须等于寄存器长度，每个值范围为0-65535。",
                        parent=win,
                    )
                    return
                seen.add(key)
                out.append(
                    {
                        "kind": k,
                        "address": int(a),
                        "mapping": int(m),
                        "length": int(l),
                        "value": defaults.copy(),
                        "default": defaults,
                        "label": str(text),
                    }
                )
            if on_saved:
                on_saved(out)
            else:
                self.dev.set_items(out)
                self.save_config()
                self.refresh_tree()
            win.destroy()

        bottom = ttk.Frame(frame)
        bottom.pack(fill="x", pady=(8, 0))
        if table_path:

            def rename_table():
                old_name = os.path.splitext(os.path.basename(table_path_holder[0]))[0]
                new_name = simpledialog.askstring(
                    "修改点表名称",
                    "请输入新的点表名称：",
                    initialvalue=old_name,
                    parent=win,
                )
                if not new_name:
                    return
                safe = "".join(c for c in new_name.strip() if c not in '\\/:*?"<>|')
                new_path = (
                    os.path.join(self.tables_dir, safe + ".json") if safe else None
                )
                if not new_path:
                    messagebox.showerror("名称错误", "点表名称不能为空。", parent=win)
                    return
                if new_path != table_path_holder[0] and os.path.exists(new_path):
                    messagebox.showerror("名称重复", "该点表名称已经存在。", parent=win)
                    return
                if new_path != table_path_holder[0]:
                    try:
                        os.rename(table_path_holder[0], new_path)
                    except Exception as e:
                        messagebox.showerror("重命名失败", str(e), parent=win)
                        return
                table_path_holder[0] = new_path
                win.title("编辑点表：" + safe)
                messagebox.showinfo("重命名成功", "点表名称已修改。", parent=win)

            ttk.Button(bottom, text="修改点表名称", command=rename_table).pack(
                side="left"
            )
        ttk.Button(bottom, text="保存配置", style="Accent.TButton", command=save).pack(
            side="right"
        )

    def refresh_tree(self):
        for i in self.tree.get_children():
            self.tree.delete(i)
        for number, x in enumerate(
            sorted(self.dev.snapshot(), key=lambda z: (z["kind"], z["address"])), 1
        ):
            self.tree.insert(
                "",
                "end",
                values=(
                    number,
                    x.get("label", ""),
                    x["kind"],
                    x["address"],
                    x.get("mapping", x["address"]),
                    x.get("length", 1),
                    (
                        ",".join(map(str, x["value"]))
                        if isinstance(x.get("value"), list)
                        else x["value"]
                    ),
                    (
                        ",".join(map(str, x["default"]))
                        if isinstance(x.get("default"), list)
                        else x["default"]
                    ),
                ),
            )

    def prepare_tables(self):
        os.makedirs(self.tables_dir, exist_ok=True)
        if not any(
            name.lower().endswith(".json") for name in os.listdir(self.tables_dir)
        ) and os.path.exists(self.config_file):
            try:
                with open(self.config_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                with open(
                    os.path.join(self.tables_dir, "默认点表.json"),
                    "w",
                    encoding="utf-8",
                ) as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
            except Exception:
                pass

    def table_files(self):
        return sorted(
            [
                os.path.join(self.tables_dir, n)
                for n in os.listdir(self.tables_dir)
                if n.lower().endswith(".json")
            ]
        )

    def normalize_items(self, items):
        for x in items:
            x.setdefault("label", "")
            x.setdefault("mapping", x.get("address", 0))
            x.setdefault("length", 1)
            if not isinstance(x.get("default"), list):
                x["default"] = [x.get("default", 0)] * int(x["length"])
            if not isinstance(x.get("value"), list):
                x["value"] = [x.get("value", x["default"][0])] * int(x["length"])
        return items

    def select_table(self):
        win = tk.Toplevel(self)
        win.title("选择从机点表")
        win.geometry("620x430")
        win.configure(bg=BG)
        win.transient(self)
        win.grab_set()
        frame = ttk.Frame(win)
        frame.pack(fill="both", expand=True, padx=18, pady=18)
        ttk.Label(frame, text="本地从机点表", style="Title.TLabel").pack(
            anchor="w", pady=(0, 12)
        )
        table_list = tk.Listbox(
            frame,
            bg=PANEL,
            fg=FG,
            selectbackground="#2563eb",
            selectforeground="white",
            font=("Segoe UI", 11),
            relief="flat",
            height=12,
        )
        table_list.pack(fill="both", expand=True)
        paths = self.table_files()
        names = [os.path.splitext(os.path.basename(p))[0] for p in paths]
        [table_list.insert("end", n) for n in names]

        def edit_path(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as e:
                messagebox.showerror("点表读取失败", str(e), parent=win)
                return
            data = self.normalize_items(data)
            path_holder = [path]

            def saved(items):
                with open(path_holder[0], "w", encoding="utf-8") as f:
                    json.dump(items, f, ensure_ascii=False, indent=2)
                win.destroy()
                self.select_table()

            self.edit(data, saved, path_holder)

        def new_table():
            name = simpledialog.askstring("新建点表", "请输入点表名称：", parent=win)
            if not name:
                return
            safe = "".join(c for c in name.strip() if c not in '\\/:*?"<>|')
            if not safe:
                return
            path = os.path.join(self.tables_dir, safe + ".json")
            if os.path.exists(path):
                messagebox.showerror("名称重复", "该点表名称已经存在。", parent=win)
                return
            with open(path, "w", encoding="utf-8") as f:
                json.dump([], f, ensure_ascii=False, indent=2)
            edit_path(path)

        def selected_path():
            selected = table_list.curselection()
            return paths[selected[0]] if selected else None

        def edit_selected():
            path = selected_path()
            if path:
                edit_path(path)

        def choose():
            path = selected_path()
            if not path:
                messagebox.showwarning(
                    "请选择点表", "请先选择一个本地点表。", parent=win
                )
                return
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.dev.set_items(self.normalize_items(data))
                self.current_table_path = path
                self.current_table_name = os.path.splitext(os.path.basename(path))[0]
                self.current_label.configure(
                    text="当前点表：" + self.current_table_name
                )
                self.refresh_tree()
                win.destroy()
            except Exception as e:
                messagebox.showerror("点表加载失败", str(e), parent=win)

        controls = ttk.Frame(frame)
        controls.pack(fill="x", pady=(12, 0))
        ttk.Button(controls, text="新建点表", command=new_table).pack(side="left")
        ttk.Button(controls, text="编辑选中", command=edit_selected).pack(
            side="left", padx=8
        )
        ttk.Button(controls, text="确定", style="Accent.TButton", command=choose).pack(
            side="right"
        )

    def save_config(self):
        if not self.current_table_path:
            return
        try:
            with open(self.current_table_path, "w", encoding="utf-8") as f:
                json.dump(self.dev.snapshot(), f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.log("配置保存失败: " + str(e))

    def load_config(self):
        self.dev.set_items([])
        self.current_table_path = None
        self.current_table_name = "未选择点表"
        self.current_label.configure(text="当前点表：未选择点表")
        self.refresh_tree()

    def log(self, msg):
        self.events.put(msg)

    def refresh(self):
        while not self.events.empty():
            msg = self.events.get()
            self.logbox.configure(state="normal")
            self.logbox.insert("end", time.strftime("%H:%M:%S ") + msg + "\n")
            self.logbox.see("end")
            self.logbox.configure(state="disabled")
        self.refresh_tree()
        self.after(200, self.refresh)

    def toggle(self):
        if self.dev.running:
            self.dev.stop()
            self.status.configure(text="● 已停止", foreground=RED)
            self.startbtn.configure(text="▶  启动模拟")
            self.configbtn.state(["!disabled"])
            self.resetbtn.state(["!disabled"])
            self.loopbtn.state(["disabled"])
            self.log("模拟已停止")
            return
        try:
            parity = {"无": "N", "奇": "O", "偶": "E"}[self.vars["parity"].get()]
            stop = float(self.vars["stop"].get())
            data = int(self.vars["data"].get())
            slave = int(self.vars["slave"].get())
            assert 1 <= slave <= 247
            self.dev.start(
                self.vars["port"].get(),
                int(self.vars["baud"].get()),
                parity,
                stop,
                data,
                slave,
                self.log,
            )
            self.status.configure(text=f"● 运行中  从机 {slave}", foreground=GREEN)
            self.startbtn.configure(text="■  停止模拟")
            self.configbtn.state(["disabled"])
            self.resetbtn.state(["disabled"])
            self.loopbtn.state(["!disabled"])
            self.log("已打开 " + self.vars["port"].get())
        except Exception as e:
            messagebox.showerror("启动失败", str(e))

    def reset_registers(self):
        self.dev.reset_values()
        self.refresh_tree()
        self.save_config()
        self.log("所有寄存器已重置为默认值")

    def loop_test(self):
        win = tk.Toplevel(self)
        win.title("本地回环测试")
        win.geometry("950x650")
        win.configure(bg=BG)
        win.transient(self)
        vars = {
            k: tk.StringVar(value=v)
            for k, v in {
                "port": "COM2",
                "baud": "9600",
                "parity": "无",
                "stop": "1",
                "data": "8",
                "slave": self.vars["slave"].get(),
                "timeout": "1000",
                "count": "10",
                "interval": "500",
            }.items()
        }
        top = ttk.Frame(win)
        top.pack(fill="x", padx=18, pady=14)
        for label, key, values in [
            ("测试串口", "port", None),
            (
                "波特率",
                "baud",
                ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"],
            ),
            ("校验", "parity", ["无", "奇", "偶"]),
            ("停止位", "stop", ["1", "2"]),
            ("数据位", "data", ["7", "8"]),
            ("从机地址", "slave", None),
            ("超时(ms)", "timeout", None),
            ("测试次数", "count", None),
            ("间隔(ms)", "interval", None),
        ]:
            box = ttk.Frame(top)
            box.pack(side="left", padx=(0, 10))
            ttk.Label(box, text=label, style="Muted.TLabel").pack(anchor="w")
            # 测试串口必须使用下拉框，刷新串口时需要更新其 values；同时允许手动输入不存在于列表中的串口。
            w = (
                ttk.Combobox(
                    box,
                    textvariable=vars[key],
                    values=values or [],
                    width=9,
                    state="normal" if key == "port" else "readonly",
                )
                if (values or key == "port")
                else ttk.Entry(box, textvariable=vars[key], width=9)
            )
            w.pack(pady=(4, 0))
            if key == "port":
                portbox = w

        def refresh_ports():
            if list_ports:
                portbox["values"] = [p.device for p in list_ports.comports()]

        ttk.Button(top, text="刷新串口", command=refresh_ports).pack(
            side="left", padx=4, pady=17
        )
        options = ttk.Frame(win, style="Card.TFrame")
        options.pack(fill="x", padx=18, pady=(0, 10))
        ttk.Label(
            options,
            text="自定义测试选项",
            font=("Segoe UI Semibold", 11),
            background=PANEL,
            foreground=FG,
        ).pack(anchor="w", padx=12, pady=(8, 4))
        option_vars = {}
        parent_vars = {}
        option_defs = {
            "线圈": ["单读", "单写", "连续读", "连续写"],
            "离散输入": ["单读", "连续读"],
            "保持寄存器": ["单读", "单写", "连续读", "连续写"],
            "输入寄存器": ["单读", "连续读"],
        }
        option_rows = ttk.Frame(options)
        option_rows.pack(fill="x", padx=12, pady=(0, 8))

        def toggle_button(parent, text, var, command=None):
            button = tk.Button(
                parent,
                bg=BG,
                fg=FG,
                activebackground="#203855",
                activeforeground=CYAN,
                relief="flat",
                bd=0,
                highlightthickness=0,
                anchor="w",
                padx=2,
                cursor="hand2",
            )

            def redraw(*_):
                button.configure(text=("● " if var.get() else "○ ") + text)

            def click():
                var.set(not var.get())
                command() if command else None

            button.configure(command=click)
            var.trace_add("write", redraw)
            redraw()
            return button

        for col, (kind, ops) in enumerate(option_defs.items()):
            group = ttk.Frame(option_rows)
            group.grid(row=0, column=col, padx=(0, 18), sticky="w")
            parent_vars[kind] = tk.BooleanVar(value=True)
            option_vars[kind] = {op: tk.BooleanVar(value=True) for op in ops}

            def cascade(k=kind):
                if not parent_vars[k].get():
                    for v in option_vars[k].values():
                        v.set(False)

            toggle_button(group, kind, parent_vars[kind], cascade).pack(
                anchor="w", fill="x"
            )
            for op in ops:
                toggle_button(group, op, option_vars[kind][op]).pack(
                    side="left", padx=(0, 6)
                )
        stat = ttk.Frame(win, style="Card.TFrame")
        stat.pack(fill="x", padx=18, pady=(0, 10))
        total = tk.StringVar(value="测试次数：0")
        success = tk.StringVar(value="成功：0")
        rate = tk.StringVar(value="成功率：0.0%")
        [
            ttk.Label(stat, textvariable=v, background=PANEL, foreground=FG).pack(
                side="left", padx=16, pady=10
            )
            for v in (total, success, rate)
        ]
        logbox = tk.Text(
            win,
            bg=PANEL,
            fg=MUTED,
            insertbackground=FG,
            relief="flat",
            state="disabled",
            wrap="word",
        )
        logbox.pack(fill="both", expand=True, padx=18, pady=(0, 10))
        controls = ttk.Frame(win)
        controls.pack(fill="x", padx=18, pady=(0, 16))
        running = {"value": False}
        worker = {"thread": None}
        stop_event = threading.Event()
        logbox.tag_configure("success", foreground=GREEN)
        logbox.tag_configure("failure", foreground=RED)

        def write_log(message, tag=None):
            def append():
                logbox.configure(state="normal")
                logbox.insert(
                    "end",
                    time.strftime("%Y-%m-%d %H:%M:%S ") + message + "\n",
                    *((tag,) if tag else ()),
                )
                logbox.see("end")
                logbox.configure(state="disabled")

            win.after(0, append)

        def update_stats(n, ok):
            win.after(
                0,
                lambda: (
                    total.set(f"测试次数：{n}"),
                    success.set(f"成功：{ok}"),
                    rate.set(f"成功率：{ok/n*100:.1f}%"),
                ),
            )

        def read_response(ser, function, quantity=1):
            first = ser.read(3)
            if len(first) < 3:
                return b""
            length = first[2] if function in (1, 2, 3, 4) else 5
            rest = ser.read(length + 2)
            return first + rest

        def send_frame(ser, frame, function, quantity=1):
            ser.reset_input_buffer()
            ser.write(frame + crc16(frame))
            ser.flush()
            return read_response(ser, function, quantity)

        def make_read(slave, function, address, quantity=1):
            return bytes([slave, function]) + struct.pack(">HH", address, quantity)

        def make_write(slave, function, address, value):
            return bytes([slave, function]) + struct.pack(">HH", address, value)

        def make_multi_write(slave, function, address, values, bits=False):
            quantity = len(values)
            payload = bytearray((quantity + 7) // 8 if bits else quantity * 2)
            if bits:
                for i, v in enumerate(values):
                    payload[i // 8] |= (int(v) & 1) << (i % 8)
            else:
                for i, v in enumerate(values):
                    payload[i * 2 : i * 2 + 2] = struct.pack(">H", int(v) & 0xFFFF)
            return (
                bytes([slave, function])
                + struct.pack(">HHB", address, quantity, len(payload))
                + bytes(payload)
            )

        def test_worker():
            try:
                parity = {"无": "N", "奇": "O", "偶": "E"}[vars["parity"].get()]
                timeout_ms = int(vars["timeout"].get())
                slave = int(vars["slave"].get())
                assert 1 <= slave <= 247 and timeout_ms > 0
                ser = serial.Serial(
                    port=vars["port"].get(),
                    baudrate=int(vars["baud"].get()),
                    parity=parity,
                    stopbits=float(vars["stop"].get()),
                    bytesize=int(vars["data"].get()),
                    timeout=timeout_ms / 1000,
                )
                items = self.dev.snapshot()
                count = int(vars["count"].get())
                delay = max(0, int(vars["interval"].get())) / 1000
                ok = 0
                if not items:
                    raise RuntimeError("当前没有配置寄存器")
                selected_ops = {
                    k: [op for op, v in option_vars[k].items() if v.get()]
                    for k in option_defs
                }
                candidates = [x for x in items if selected_ops.get(x["kind"])]
                if not candidates:
                    raise RuntimeError("没有勾选任何可用的测试项目")
                for n in range(1, count + 1):
                    if stop_event.is_set():
                        break
                    item = random.choice(candidates)
                    group_items = [item]
                    kind = item["kind"]
                    operation = random.choice(selected_ops[kind])
                    address = int(item.get("mapping", item["address"]))
                    length = int(item.get("length", 1))
                    is_bit = kind in ("线圈", "离散输入")
                    do_write = operation in ("单写", "连续写")
                    do_multi = length > 1
                    # “连续读/连续写”才尝试合并相邻配置项；单独操作一个长度为N的项目，仍按N个寄存器生成多寄存器帧。
                    if operation in ("连续读", "连续写"):
                        group = [item]
                        next_addr = address + length
                        for candidate in sorted(
                            items, key=lambda x: int(x.get("mapping", x["address"]))
                        ):
                            caddr = int(candidate.get("mapping", candidate["address"]))
                            clen = int(candidate.get("length", 1))
                            if (
                                candidate is not item
                                and candidate["kind"] == kind
                                and caddr == next_addr
                            ):
                                group.append(candidate)
                                next_addr += clen
                        if len(group) > 1:
                            group_items = group
                            length = sum(int(x.get("length", 1)) for x in group)
                            do_multi = length > 1
                            item = dict(item)
                            item["label"] = "+".join(
                                str(x.get("label", "")) for x in group if x.get("label")
                            )

                            def flatten_value(x, key):
                                value = x.get(key, 0)
                                return (
                                    value
                                    if isinstance(value, list)
                                    else [value] * int(x.get("length", 1))
                                )

                            item["value"] = [
                                v for x in group for v in flatten_value(x, "value")
                            ]
                            item["default"] = [
                                v for x in group for v in flatten_value(x, "default")
                            ]
                    fn = (
                        (15 if kind == "线圈" else 16)
                        if do_write and do_multi
                        else (
                            (5 if kind == "线圈" else 6)
                            if do_write
                            else {
                                "线圈": 1,
                                "离散输入": 2,
                                "保持寄存器": 3,
                                "输入寄存器": 4,
                            }[kind]
                        )
                    )
                    if do_write:
                        expected = (
                            [
                                (
                                    random.randint(0, 1)
                                    if is_bit
                                    else random.randint(0, 65535)
                                )
                                for _ in range(length)
                            ]
                            if do_multi
                            else (
                                random.randint(0, 1)
                                if is_bit
                                else random.randint(0, 65535)
                            )
                        )
                        frame = (
                            make_multi_write(slave, fn, address, expected, is_bit)
                            if do_multi
                            else make_write(
                                slave,
                                fn,
                                address,
                                (0xFF00 if expected else 0) if fn == 5 else expected,
                            )
                        )
                    else:
                        base = item.get("value", item.get("default", 0))
                        base_values = base if isinstance(base, list) else [int(base)]
                        expected = (base_values + [0] * length)[:length]
                        frame = make_read(slave, fn, address, length)
                    full_frame = frame + crc16(frame)
                    response = send_frame(ser, frame, fn, length)
                    actual = None
                    valid_crc = len(response) >= 5 and response[-2:] == crc16(
                        response[:-2]
                    )
                    exception_code = (
                        response[2]
                        if len(response) >= 5
                        and valid_crc
                        and response[0] == slave
                        and response[1] == (fn | 0x80)
                        else None
                    )
                    valid = (
                        valid_crc
                        and len(response) >= 5
                        and response[0] == slave
                        and response[1] == fn
                    )
                    if valid:
                        if do_write:
                            actual = expected
                        elif is_bit:
                            actual = [
                                (response[3 + i // 8] >> (i % 8)) & 1
                                for i in range(length)
                            ]
                        else:
                            actual = [
                                struct.unpack(">H", response[3 + i * 2 : 5 + i * 2])[0]
                                for i in range(length)
                            ]
                    passed = valid and actual == expected
                    ok += 1 if passed else 0
                    error_code = (
                        "0"
                        if passed
                        else (
                            f"{exception_code:02X}"
                            if exception_code is not None
                            else (
                                "无响应"
                                if not response
                                else (
                                    "CRC错误"
                                    if not valid_crc
                                    else ("值不一致" if valid else "协议错误")
                                )
                            )
                        )
                    )
                    if passed and do_write:
                        if len(group_items) == 1:
                            group_items[0]["value"] = expected
                        else:
                            offset = 0
                            for target in group_items:
                                size = int(target.get("length", 1))
                                target["value"] = list(expected[offset : offset + size])
                                offset += size
                    result_tag = "success" if passed else "failure"
                    update_stats(n, ok)
                    write_log(f'发送帧：{full_frame.hex(" ").upper()}', result_tag)
                    write_log(
                        f'返回帧：{response.hex(" ").upper() if response else "无响应"}',
                        result_tag,
                    )
                    write_log(
                        f'第{n}次 | {"写入" if do_write else "读取"} | 标签={item.get("label","")} | 映射地址={address} | 期望值={expected} | 实际值={actual if actual is not None else "无响应"} | 错误码={error_code} | {"成功" if passed else "失败"}',
                        result_tag,
                    )
                    time.sleep(delay)
                ser.close()
                write_log("测试完成")
            except Exception as e:
                write_log("测试错误：" + str(e))
            finally:
                running["value"] = False
                start.state(["!disabled"])
                stop.state(["disabled"])

        def start_test():
            if serial is None:
                messagebox.showerror("缺少依赖", "请先安装 pyserial。", parent=win)
                return
            try:
                assert (
                    int(vars["count"].get()) > 0
                    and int(vars["interval"].get()) >= 0
                    and 1 <= int(vars["slave"].get()) <= 247
                    and int(vars["timeout"].get()) > 0
                )
            except:
                messagebox.showerror(
                    "参数错误",
                    "从机地址必须为1-247；超时时间必须大于0；测试次数必须大于0；间隔必须是不小于0的整数。",
                    parent=win,
                )
                return
            running["value"] = True
            stop_event.clear()
            start.state(["disabled"])
            stop.state(["!disabled"])
            worker["thread"] = threading.Thread(target=test_worker, daemon=True)
            worker["thread"].start()

        def stop_test():
            stop_event.set()
            write_log("正在停止测试……")

        start = ttk.Button(
            controls, text="开始测试", style="Accent.TButton", command=start_test
        )
        start.pack(side="left")
        stop = ttk.Button(
            controls, text="停止测试", command=stop_test, state="disabled"
        )
        stop.pack(side="left", padx=8)
        ttk.Button(
            controls, text="关闭", command=lambda: (stop_event.set(), win.destroy())
        ).pack(side="right")
        refresh_ports()

    def close(self):
        self.save_config()
        self.dev.stop()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()

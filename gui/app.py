"""
gui.py
GUI Tkinter + aggiornamento live della simulazione FFSCC.
Layout: colonna sinistra = sidebar telemetria/controlli
        colonna destra  = 5 pannelli temporali (sinistra) + schema motore (destra)
"""
import tkinter as tk
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as mpatches

from core.thermodynamics import CH4RealGasProps, CEA_MethaloxCombustion
from core.engine import FFSCC_Engine
from core.avionics import FlightComputer
from config import K_HEAD
from core.nozzle import SpacialNozzleModel
from core.structures import StructuralAnalyzer

# ── Palette colori ─────────────────────────────────────────────────────────────
BG_FIG   = '#0d1b2a'
BG_AX    = '#1a2b3c'
COL_GRID = '#2c3e50'
COL_TEXT = '#ecf0f1'

C_THRUST  = '#00cec9'
C_TARGET  = '#fdcb6e'
C_MCC     = '#a29bfe'
C_PORHC   = '#55efc4'
C_PFRHC   = '#fd79a8'
C_OF_ORHC = '#00b894'
C_OF_FRHC = '#e84393'
C_OX_RPM  = '#74b9ff'
C_F_RPM   = '#55efc4'
C_TCOOL   = '#ff7675'
C_LOX_TK  = '#00cec9'
C_CH4_TK  = '#a29bfe'
C_MOV     = '#0984e3'
C_MFV     = '#d63031'
C_AUTOX   = '#74b9ff'
C_AUTOF   = '#55efc4'
C_MDOT_OX = '#fdcb6e'
C_MDOT_F  = '#fd79a8'


def _style_ax(ax, xlabel=None, ylabel=None, title=None,
              ylim=None, ylabel_color=COL_TEXT):
    ax.set_facecolor(BG_AX)
    for spine in ax.spines.values():
        spine.set_edgecolor(COL_GRID)
    ax.tick_params(colors=COL_TEXT, labelsize=10)
    ax.xaxis.label.set_color(COL_TEXT)
    ax.yaxis.label.set_color(ylabel_color)
    ax.grid(True, color=COL_GRID, linewidth=0.6, alpha=0.8)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=10, color=COL_TEXT)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=10, color=ylabel_color)
    if title:
        ax.set_title(title, fontsize=11, fontweight='bold',
                     color=COL_TEXT, pad=5)
    if ylim:
        ax.set_ylim(*ylim)


def _style_twin(ax, ylabel, ylabel_color, ylim=None):
    ax.tick_params(colors=ylabel_color, labelsize=10)
    ax.yaxis.label.set_color(ylabel_color)
    ax.set_ylabel(ylabel, fontsize=10, color=ylabel_color)
    for spine in ax.spines.values():
        spine.set_edgecolor(COL_GRID)
    if ylim:
        ax.set_ylim(*ylim)


def _pcol(p_bar, p_max=800.0):
    """Mappa pressione → colore: blu scuro (bassa) → ciano → giallo → rosso (alta)."""
    import matplotlib.cm as _cm
    t = float(np.clip(p_bar / p_max, 0.0, 1.0))
    r, g, b, _ = _cm.plasma(t)
    return (r, g, b)


class AppGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("FFSCC Digital Twin — Full Pressure Ladder  v2.1")
        self.root.configure(bg=BG_FIG)
        self.root.geometry("1700x1050")

        self.engine = FFSCC_Engine()
        self.dt     = 0.05
        self.fc     = FlightComputer(dt=self.dt)
        self.target_thrust = 2750.0

        self.max_len = 400
        self.hist = {k: [0.0] * self.max_len for k in [
            't', 'th', 'th_tgt', 'p_mcc',
            'p_orhc', 'p_frhc', 'of_orhc', 'of_frhc',
            'w_ox', 'w_f', 't_cool',
            'p_tank_ox', 'p_tank_f',
            'v_mov', 'v_mfv', 'v_orhc', 'v_frhc', 'v_auto_ox', 'v_auto_f',
            'mdot_ox', 'mdot_f',
        ]}

        self.spatial_model  = SpacialNozzleModel()
        self.struct_analyzer = StructuralAnalyzer()
        self._right_panel   = "SCHEMA"   # "SCHEMA" | "NOZZLE" | "STRUCT"

        self._build_sidebar(root)

        plot_frame = tk.Frame(root, bg=BG_FIG)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self._build_figure(plot_frame)
        self._rebuild_nozzle_artists()   # inizializza linee (poi subito sovrascritte da schema al boot)
        self.root.after(50, self._update_sim)

    # ── Sidebar ───────────────────────────────────────────────────────────────
    def _build_sidebar(self, root):
        ctrl = tk.Frame(root, width=330, bg=BG_FIG)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0), pady=10)
        ctrl.pack_propagate(False)

        self.lbl_state = tk.Label(
            ctrl, text="STATE: IDLE", fg="black", bg="#4a4a4a",
            font=("Courier", 16, "bold"), relief="flat", pady=6,
        )
        self.lbl_state.pack(pady=(14, 2), padx=10, fill=tk.X)

        self.lbl_abort = tk.Label(ctrl, text="", fg="#ff7675", bg=BG_FIG,
                                  font=("Courier", 11, "bold"))
        self.lbl_abort.pack(pady=(0, 4))

        self._tel_block(ctrl, "PERFORMANCE", "#f39c12")
        self.lbl_tel_perf = self._tel_label(ctrl, "#ffffff")

        self._tel_separator(ctrl, "PRE-BURNERS", "#ff7675")
        self.lbl_tel_pb = self._tel_label(ctrl, "#ff7675")

        self._tel_separator(ctrl, "TURBOPOMPE / SERBATOI", "#74b9ff")
        self.lbl_tel_sys = self._tel_label(ctrl, "#74b9ff")

        self._tel_separator(ctrl, "MASS FLOW  /  O/F MCC", "#55efc4")
        self.lbl_tel_mdot = self._tel_label(ctrl, "#55efc4")

        self._tel_separator(ctrl, "VALVOLE", "#b2bec3")
        self.lbl_tel_v = self._tel_label(ctrl, "#b2bec3")

        tk.Frame(ctrl, bg=COL_GRID, height=1).pack(fill=tk.X, padx=10, pady=10)

        tk.Button(
            ctrl, text="▶   START IGNITION", bg="#27ae60", fg="black",
            font=("Arial", 13, "bold"), command=self.fc.start_sequence,
            relief="flat", activebackground="#2ecc71", activeforeground="black", pady=8,
        ).pack(fill=tk.X, padx=10, pady=(0, 6))

        tk.Label(ctrl, text="TARGET THRUST (kN)", fg="#b2bec3",
                 bg=BG_FIG, font=("Arial", 11, "bold")).pack(pady=(4, 0))
        self.slider = tk.Scale(
            ctrl, from_=3200, to=800, orient=tk.HORIZONTAL,
            bg=BG_FIG, fg=COL_TEXT, troughcolor="#2c3e50",
            highlightthickness=0, font=("Arial", 10),
            command=lambda v: setattr(self, 'target_thrust', float(v)),
        )
        self.slider.set(2750)
        self.slider.pack(fill=tk.X, padx=10)

        tk.Frame(ctrl, bg=COL_GRID, height=1).pack(fill=tk.X, padx=10, pady=8)

        btn_row = tk.Frame(ctrl, bg=BG_FIG)
        btn_row.pack(fill=tk.X, padx=10, pady=(0, 4))
        tk.Button(btn_row, text="⏹  MECO", bg="#8e44ad", fg="black",
                  font=("Arial", 12, "bold"), command=self.fc.meco,
                  relief="flat", pady=7,
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        tk.Button(btn_row, text="⚠  ABORT", bg="#c0392b", fg="black",
                  font=("Arial", 12, "bold"),
                  command=lambda: self.fc.abort("MANUAL"),
                  relief="flat", pady=7,
                  ).pack(side=tk.LEFT, expand=True, fill=tk.X)

        # ── Toggle pannello destro ─────────────────────────────────────────────
        tk.Frame(ctrl, bg=COL_GRID, height=1).pack(fill=tk.X, padx=10, pady=(10, 4))
        tk.Label(ctrl, text="PANNELLO DESTRO", fg="#b2bec3",
                 bg=BG_FIG, font=("Arial", 10, "bold")).pack(pady=(0, 4))
        view_row = tk.Frame(ctrl, bg=BG_FIG)
        view_row.pack(fill=tk.X, padx=10)
        self._btn_schema = tk.Button(
            view_row, text="⚙  SCHEMA", bg="#2980b9", fg="black",
            font=("Arial", 11, "bold"), relief="flat", pady=6,
            command=self._show_schema,
        )
        self._btn_schema.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self._btn_nozzle = tk.Button(
            view_row, text="🌡  NOZZLE", bg="#4a4a4a", fg="black",
            font=("Arial", 11, "bold"), relief="flat", pady=6,
            command=self._show_nozzle,
        )
        self._btn_nozzle.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4, 0))

        struct_row = tk.Frame(ctrl, bg=BG_FIG)
        struct_row.pack(fill=tk.X, padx=10, pady=(4, 0))
        self._btn_struct = tk.Button(
            struct_row, text="🔩  STRUTTURA", bg="#4a4a4a", fg="black",
            font=("Arial", 11, "bold"), relief="flat", pady=6,
            command=self._show_struct,
        )
        self._btn_struct.pack(fill=tk.X)

    def _tel_block(self, parent, title, color):
        tk.Label(parent, text=f"── {title} ──", fg=color, bg=BG_FIG,
                 font=("Arial", 10, "bold")).pack(anchor="w", padx=12, pady=(7, 1))

    def _tel_separator(self, parent, title, color):
        self._tel_block(parent, title, color)

    def _tel_label(self, parent, color):
        lbl = tk.Label(parent, text="", fg=color, bg=BG_FIG,
                       font=("Courier", 11), justify=tk.LEFT)
        lbl.pack(anchor="w", padx=18, pady=1)
        return lbl

    def _rebuild_nozzle_artists(self):
        """Ricrea le linee del profilo termico su ax_schema (necessario dopo ax.clear())."""
        ax = self.ax_schema
        ax.clear()
        _style_ax(ax, xlabel="Coordinata assiale (m)", ylabel="Temperatura (K)",
                  title="PROFILO TERMICO UGELLO 1D", ylim=(0, 4200))
        ax.set_xlim(-0.2, 0.6)
        self.l_gas,     = ax.plot([], [], lw=1.5, color='#e17055', alpha=0.4, label="T Gas")
        self.l_hw_hot,  = ax.plot([], [], lw=2.5, color='#d35400', label="T Rame ← gas")
        self.l_hw_cold, = ax.plot([], [], lw=2.5, color='#f39c12', linestyle='--', label="T Rame ← CH4")
        self.l_cool,    = ax.plot([], [], lw=2.5, color='#74b9ff', label="T Metano")
        self.l_cw,      = ax.plot([], [], lw=2,   color='#b2bec3', label="T Acciaio")
        ax.axhline(y=1356.0, color='#e17055', linestyle=':', lw=1.2, label="T fusione rame")
        ax.axvline(x=0.0,    color='#636e72', linestyle='--', alpha=0.6, lw=1, label="Gola")
        ax.legend(loc="upper right", fontsize=9, facecolor=BG_AX,
                  labelcolor=COL_TEXT, framealpha=0.8)

    def _show_schema(self):
        self._right_panel = "SCHEMA"
        self._btn_schema.config(bg="#2980b9")
        self._btn_nozzle.config(bg="#4a4a4a")
        self._btn_struct.config(bg="#4a4a4a")

    def _show_nozzle(self):
        self._right_panel = "NOZZLE"
        self._btn_schema.config(bg="#4a4a4a")
        self._btn_nozzle.config(bg="#16a085")
        self._btn_struct.config(bg="#4a4a4a")
        self._rebuild_nozzle_artists()   # ricostruisce dopo eventuale ax.clear() dello schema

    def _show_struct(self):
        self._right_panel = "STRUCT"
        self._btn_schema.config(bg="#4a4a4a")
        self._btn_nozzle.config(bg="#4a4a4a")
        self._btn_struct.config(bg="#8e44ad")

    # ── Figure ────────────────────────────────────────────────────────────────
    def _build_figure(self, plot_frame):
        self.fig = Figure(figsize=(15, 10), dpi=96)
        self.fig.patch.set_facecolor(BG_FIG)
        self.fig.subplots_adjust(
            hspace=0.45, wspace=0.38,
            left=0.06, right=0.96, top=0.97, bottom=0.05,
        )

        gs = self.fig.add_gridspec(5, 3)

        self.ax_t     = self.fig.add_subplot(gs[0, :2])
        self.ax_pb    = self.fig.add_subplot(gs[1, :2])
        self.ax_w     = self.fig.add_subplot(gs[2, :2])
        self.ax_tank  = self.fig.add_subplot(gs[3, :2])
        self.ax_v     = self.fig.add_subplot(gs[4, :2])
        self.ax_schema = self.fig.add_subplot(gs[:, 2])   # schema motore

        # Panel 0: Spinta e P_MCC
        self.ax_pmcc = self.ax_t.twinx()
        _style_ax(self.ax_t,  ylabel="Spinta (kN)", title="SPINTA  &  PRESSIONE CAMERA",
                  ylim=(-50, 3500), ylabel_color=C_THRUST)
        _style_twin(self.ax_pmcc, "P_MCC (bar)", C_MCC, ylim=(0, 500))
        self.l_th,   = self.ax_t.plot([], [], lw=2.5, color=C_THRUST, label="Spinta (kN)")
        self.l_tgt,  = self.ax_t.plot([], [], lw=1.5, color=C_TARGET, linestyle='--', label="Target (kN)")
        self.l_pmcc, = self.ax_pmcc.plot([], [], lw=2, color=C_MCC, label="P_MCC (bar)")
        self.ax_t.legend(loc="upper left",    fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)
        self.ax_pmcc.legend(loc="upper right", fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)

        # Panel 1: Pre-burner P + O/F
        self.ax_pb_of = self.ax_pb.twinx()
        _style_ax(self.ax_pb,  ylabel="Pressione (bar)", title="PRE-BURNER  —  PRESSIONE  &  O/F",
                  ylim=(0, 1200), ylabel_color=COL_TEXT)
        _style_twin(self.ax_pb_of, "O/F  [–]", C_OF_ORHC, ylim=(0, 80))
        self.l_porhc,   = self.ax_pb.plot([], [], lw=2,   color=C_PORHC,   label="P ORHC (bar)")
        self.l_pfrhc,   = self.ax_pb.plot([], [], lw=2,   color=C_PFRHC,   label="P FRHC (bar)")
        self.l_of_orhc, = self.ax_pb_of.plot([], [], lw=1.8, color=C_OF_ORHC, linestyle='--', label="O/F ORHC")
        self.l_of_frhc, = self.ax_pb_of.plot([], [], lw=1.8, color=C_OF_FRHC, linestyle='--', label="O/F FRHC")
        self.ax_pb.legend(loc="upper left",    fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)
        self.ax_pb_of.legend(loc="upper right", fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)

        # Panel 2: RPM + T_cool
        self.ax_cool = self.ax_w.twinx()
        _style_ax(self.ax_w,   ylabel="RPM", title="TURBOPOMPE  —  RPM  &  T REFRIGERANTE",
                  ylim=(0, 35000), ylabel_color=COL_TEXT)
        _style_twin(self.ax_cool, "T_cool (K)", C_TCOOL, ylim=(100, 1200))
        self.l_wox,   = self.ax_w.plot([], [], lw=2.2, color=C_OX_RPM, label="Ox RPM")
        self.l_wf,    = self.ax_w.plot([], [], lw=2.2, color=C_F_RPM,  label="Fuel RPM")
        self.l_tcool, = self.ax_cool.plot([], [], lw=1.8, color=C_TCOOL, label="T_cool (K)")
        self.ax_w.legend(loc="upper left",    fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)
        self.ax_cool.legend(loc="upper right", fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)

        # Panel 3: Serbatoi
        _style_ax(self.ax_tank, ylabel="Pressione (bar)", title="PRESSIONE SERBATOI", ylim=(0, 9))
        self.l_ptank_ox, = self.ax_tank.plot([], [], lw=2.5, color=C_LOX_TK, label="P Serbatoio LOX")
        self.l_ptank_f,  = self.ax_tank.plot([], [], lw=2.5, color=C_CH4_TK, label="P Serbatoio LCH4")
        self.ax_tank.axhline(y=4.0, color='#636e72', linestyle=':', lw=1.2, label="Target autogeno")
        self.ax_tank.axhline(y=2.5, color='#e17055', linestyle='--', lw=1.2, label="Limite cavitazione")
        self.ax_tank.legend(loc="upper right", fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)

        # Panel 4: Valvole + portate
        self.ax_mdot = self.ax_v.twinx()
        _style_ax(self.ax_v,   ylabel="Apertura (%)", title="VALVOLE  &  PORTATE MASSICHE", ylim=(-5, 115))
        _style_twin(self.ax_mdot, "Portata (kg/s)", C_MDOT_OX, ylim=(0, 1500))
        self.l_v_mov,     = self.ax_v.plot([], [], lw=2,   color=C_MOV,   label="MOV (Ox)")
        self.l_v_mfv,     = self.ax_v.plot([], [], lw=2,   color=C_MFV,   label="MFV (Fuel)")
        self.l_v_auto_ox, = self.ax_v.plot([], [], lw=1.4, color=C_AUTOX, linestyle=':', label="V_Auto LOX")
        self.l_v_auto_f,  = self.ax_v.plot([], [], lw=1.4, color=C_AUTOF, linestyle=':', label="V_Auto LCH4")
        self.l_mdot_ox,   = self.ax_mdot.plot([], [], lw=2, color=C_MDOT_OX, label="ṁ LOX (kg/s)")
        self.l_mdot_f,    = self.ax_mdot.plot([], [], lw=2, color=C_MDOT_F,  label="ṁ CH4 (kg/s)")
        self.ax_v.legend(loc="upper left",    fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)
        self.ax_mdot.legend(loc="upper right", fontsize=9, facecolor=BG_AX, labelcolor=COL_TEXT, framealpha=0.8)
        self.ax_v.set_xlabel("Tempo (s)", fontsize=8, color=COL_TEXT)

        # Schema motore (colonna destra, riempie tutto)
        self.ax_schema.set_facecolor(BG_FIG)
        self.ax_schema.axis('off')

        # Linee nozzle create da _rebuild_nozzle_artists (chiamata dopo)
        self.l_gas = self.l_hw_hot = self.l_hw_cold = self.l_cool = self.l_cw = None

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._time_axes = [self.ax_t, self.ax_pb, self.ax_w, self.ax_tank, self.ax_v]

    # ── Schema motore ─────────────────────────────────────────────────────────
    def _draw_schema(self, st, engine, state_str):
        ax = self.ax_schema
        ax.clear()
        ax.set_facecolor('#e8edf2')   # sfondo chiaro per testo nero leggibile
        ax.set_xlim(0, 8)
        ax.set_ylim(0, 16)
        ax.axis('off')

        # ── Valori correnti ───────────────────────────────────────────────────
        p_mcc      = st[0] / 1e5
        w_ox       = st[1]
        w_f        = st[2]
        t_cool     = st[7]
        p_tank_ox  = st[8]
        p_tank_f   = st[9]
        p_orhc     = st[12] / 1e5
        p_frhc     = st[13] / 1e5
        t_orhc     = engine.t_orhc_current
        t_frhc     = engine.t_frhc_current
        of_orhc    = engine.of_orhc_current
        of_frhc    = engine.of_frhc_current
        mdot_ox    = engine.mdot_ox_last
        mdot_f     = engine.mdot_f_last
        of_mcc     = mdot_ox / max(mdot_f, 0.01)
        curr_thrust = engine.get_current_thrust()

        # Pressioni pompe (da formula engine.py)
        rho_ox = 1141.0
        rho_f  = CH4RealGasProps.density(engine.t_tank_f, p_tank_f)
        p_in_ox = p_tank_ox + (rho_ox * 9.81 * engine.g_force * engine.h_tank_ox_m) / 1e5
        p_in_f  = p_tank_f  + (rho_f  * 9.81 * engine.g_force * engine.h_tank_f_m)  / 1e5
        p_dh_ox = p_in_ox + K_HEAD * w_ox**2   # testa pompa LOX [bar]
        p_dh_f  = p_in_f  + K_HEAD * w_f**2    # testa pompa CH4 [bar]

        # Salto di pressione iniettori MCC (calcolato nell'engine)
        dp_inj_ox = engine.dp_inj_ox_bar
        dp_inj_f  = engine.dp_inj_f_bar
        p_man_ox  = p_mcc + dp_inj_ox
        p_man_f   = p_mcc + dp_inj_f

        # Salto di pressione iniettori pre-burner (calcolato nell'engine)
        dp_inj_ox_orhc = getattr(engine, 'dp_inj_ox_orhc_bar', 0.0)
        dp_inj_f_frhc  = getattr(engine, 'dp_inj_f_frhc_bar',  0.0)
        p_man_ox_pb    = getattr(engine, 'p_man_ox_bar', p_dh_ox)
        p_man_f_pb     = getattr(engine, 'p_man_f_bar',  p_dh_f)

        # ── Helper: disegna box con colore pressione ───────────────────────────
        def box(x, y, w, h, title, lines, p_bar, edge='#4a6080'):
            fc = _pcol(p_bar, p_max=800)
            brightness = float(np.clip(p_bar / 400.0, 0.0, 1.0))
            ec = (brightness, float(np.clip(brightness * 0.8 + 0.2, 0, 1)),
                  float(np.clip(1.0 - brightness * 0.6, 0, 1)))
            patch = mpatches.FancyBboxPatch(
                (x, y), w, h,
                boxstyle="round,pad=0.15",
                facecolor=fc, edgecolor=ec,
                linewidth=2.0, zorder=3, alpha=0.92
            )
            ax.add_patch(patch)
            ax.text(x + w/2, y + h - 0.2, title,
                    ha='center', va='top', fontsize=9, fontweight='bold',
                    color='black', zorder=4)
            ax.text(x + w/2, y + h/2 - 0.15, '\n'.join(lines),
                    ha='center', va='center', fontsize=8.5,
                    color='black', zorder=4, family='monospace',
                    linespacing=1.5)
            ax.text(x + w - 0.12, y + 0.2, f'{p_bar:.0f} bar',
                    ha='right', va='bottom', fontsize=7.5,
                    color='#1a1a1a', zorder=4, fontweight='bold')

        # ── Helper: freccia di flusso ──────────────────────────────────────────
        def arrow(x0, y0, x1, y1, color='#7f8c8d', lw=1.5, rad=0.0, lbl='', lbl_col='#2c3e50'):
            ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                        arrowprops=dict(
                            arrowstyle='->', color=color, lw=lw,
                            connectionstyle=f'arc3,rad={rad}',
                        ), zorder=2)
            if lbl:
                mx, my = (x0+x1)/2, (y0+y1)/2
                ax.text(mx + 0.15, my, lbl, ha='left', va='center',
                        fontsize=7.5, color=lbl_col, zorder=4, fontweight='bold')

        # ── Helper: indicatore valvola ─────────────────────────────────────────
        def valve(cx, cy, pct, label):
            pct_c = float(np.clip(pct, 0, 100))
            r = float(np.clip(1.0 - pct_c/100, 0, 1))
            g = float(np.clip(pct_c/100, 0, 1))
            fc = (r * 0.85, g * 0.85, 0.05)
            circ = mpatches.Circle((cx, cy), 0.36, facecolor=fc,
                                   edgecolor='#1a1a1a', linewidth=1.5, zorder=5)
            ax.add_patch(circ)
            ax.text(cx, cy + 0.02, f'{pct_c:.0f}%',
                    ha='center', va='center', fontsize=7.5,
                    color='white', fontweight='bold', zorder=6)
            ax.text(cx, cy - 0.55, label,
                    ha='center', va='top', fontsize=8,
                    color='#1a1a1a', zorder=4, fontweight='bold')

        # ── Titolo ─────────────────────────────────────────────────────────────
        ax.text(4.0, 15.65, 'PRESSURE  LADDER', ha='center', va='top',
                fontsize=12, fontweight='bold', color='#1a1a1a')
        ax.text(1.5, 15.3, '◀  OX  SIDE', ha='center', va='top',
                fontsize=9, color='#0055aa', fontweight='bold')
        ax.text(6.5, 15.3, 'FUEL  SIDE  ▶', ha='center', va='top',
                fontsize=9, color='#007755', fontweight='bold')

        # ── LOX TANK  /  LCH4 TANK ────────────────────────────────────────────
        box(0.2, 13.2, 3.0, 1.6, 'LOX  TANK',
            [f'P = {p_tank_ox:.2f} bar', f'T = {engine.t_tank_ox:.0f} K'],
            p_tank_ox)
        box(4.8, 13.2, 3.0, 1.6, 'LCH4  TANK',
            [f'P = {p_tank_f:.2f} bar', f'T = {engine.t_tank_f:.0f} K'],
            p_tank_f)

        arrow(1.7, 13.2, 1.7, 12.15, color='#336699', lw=2.0)
        arrow(6.3, 13.2, 6.3, 12.15, color='#227755', lw=2.0)

        # ── TP_OX  /  TP_FUEL ─────────────────────────────────────────────────
        box(0.2, 10.4, 3.0, 1.7, 'TURBOPOMPA  OX',
            [f'RPM  {w_ox:.0f}', f'P_pump {p_dh_ox:.0f} bar', f'P_ORHC {p_orhc:.0f} bar'],
            p_dh_ox)
        box(4.8, 10.4, 3.0, 1.7, 'TURBOPOMPA  FUEL',
            [f'RPM  {w_f:.0f}', f'P_pump {p_dh_f:.0f} bar', f'P_FRHC {p_frhc:.0f} bar'],
            p_dh_f)

        # valvole MOV / MFV
        valve(1.7, 9.5, st[3]*100, 'MOV')
        valve(6.3, 9.5, st[4]*100, 'MFV')

        arrow(1.7, 10.4, 1.7, 9.88, color='#336699', lw=2.0)
        arrow(6.3, 10.4, 6.3, 9.88, color='#227755', lw=2.0)
        arrow(1.7, 9.14, 1.7, 8.85, color='#336699', lw=2.0,
              lbl=f'{p_man_ox_pb:.0f} bar', lbl_col='#003388')
        arrow(6.3, 9.14, 6.3, 8.85, color='#227755', lw=2.0,
              lbl=f'{p_man_f_pb:.0f} bar', lbl_col='#005533')

        # cross-bleed
        v_frhc_pct = st[6] * 100
        v_orhc_pct = st[5] * 100
        cb_frhc = float(np.clip(st[6], 0, 1))
        cb_orhc = float(np.clip(st[5], 0, 1))
        arrow(3.2, 10.95, 4.8, 8.3,
              color=(0.2, 0.4, float(np.clip(0.4 + cb_frhc * 0.6, 0, 1))),
              lw=1.2 + 2.5 * cb_frhc, rad=-0.3,
              lbl=f'v_frhc {v_frhc_pct:.0f}%', lbl_col='#003388')
        arrow(4.8, 10.95, 3.2, 8.3,
              color=(float(np.clip(0.1 + cb_orhc * 0.3, 0, 1)),
                     float(np.clip(0.5 + cb_orhc * 0.4, 0, 1)), 0.2),
              lw=1.2 + 2.5 * cb_orhc, rad=0.3,
              lbl=f'v_orhc {v_orhc_pct:.0f}%', lbl_col='#005533')

        # jacket label
        ax.text(6.3, 9.0, f'JACKET\nT={t_cool:.0f} K',
                ha='center', va='center', fontsize=8,
                color='#8B0000', zorder=4, fontweight='bold')

        # ── ORHC  /  FRHC ─────────────────────────────────────────────────────
        box(0.2, 5.8, 3.0, 2.4, 'ORHC  (ox-rich)',
            [f'P  {p_orhc:.0f} bar',
             f'T  {t_orhc:.0f} K',
             f'O/F  {of_orhc:.1f}'],
            p_orhc)
        box(4.8, 5.8, 3.0, 2.4, 'FRHC  (fuel-rich)',
            [f'P  {p_frhc:.0f} bar',
             f'T  {t_frhc:.0f} K',
             f'O/F  {of_frhc:.2f}'],
            p_frhc)

        # ── Helper box piccolo per turbina e iniettori ────────────────────────
        def small_box(cx, cy, title, dp_bar, p_ref, color_edge, color_text, w=1.35, h=0.90):
            pct = dp_bar / max(p_ref, 1.0) * 100.0
            if 10.0 <= pct <= 20.0:
                fc = '#d4efdf'
            elif 5.0 <= pct < 10.0 or 20.0 < pct <= 30.0:
                fc = '#fef9e7'
            else:
                fc = '#fadbd8'
            patch = mpatches.FancyBboxPatch(
                (cx - w/2, cy - h/2), w, h,
                boxstyle="round,pad=0.08",
                facecolor=fc, edgecolor=color_edge,
                linewidth=1.8, zorder=5, alpha=0.95
            )
            ax.add_patch(patch)
            ax.text(cx, cy + h/2 - 0.17, title,
                    ha='center', va='center', fontsize=6.5,
                    color='#333333', zorder=6, fontweight='bold')
            ax.text(cx, cy,              f'ΔP = {dp_bar:.0f} bar',
                    ha='center', va='center', fontsize=7.5,
                    color=color_text, zorder=6, family='monospace')
            ax.text(cx, cy - h/2 + 0.17, f'({pct:.1f}%  P_ref)',
                    ha='center', va='center', fontsize=7.0,
                    color=color_text, zorder=6, fontweight='bold')

        # Salti di pressione turbina
        dp_turb_ox = max(p_orhc - p_man_ox, 0.0)
        dp_turb_f  = max(p_frhc - p_man_f,  0.0)

        # ── Box INIETTORI PRE-BURNER (valve outlet → preburner) ───────────────
        small_box(1.7, 8.55, 'INJ LOX→ORHC', dp_inj_ox_orhc, p_orhc, '#336699', '#003388', w=1.4, h=0.65)
        small_box(6.3, 8.55, 'INJ CH4→FRHC', dp_inj_f_frhc,  p_frhc, '#227755', '#005533', w=1.4, h=0.65)
        arrow(1.7, 8.22, 1.7, 8.10, color='#336699', lw=2.0)
        arrow(6.3, 8.22, 6.3, 8.10, color='#227755', lw=2.0)

        # ── Frecce ORHC → TURBINA ─────────────────────────────────────────────
        arrow(1.9, 5.8, 1.55, 5.25, color='#336699', lw=2.2,
              lbl=f'{p_orhc:.0f} bar', lbl_col='#003388')
        arrow(6.1, 5.8, 6.45, 5.25, color='#227755', lw=2.2,
              lbl=f'{p_frhc:.0f} bar', lbl_col='#005533')

        # ── Box TURBINA ────────────────────────────────────────────────────────
        small_box(1.55, 4.82, 'TURBINA  OX', dp_turb_ox, p_orhc, '#336699', '#003388')
        small_box(6.45, 4.82, 'TURBINA FUEL', dp_turb_f,  p_frhc, '#227755', '#005533')

        # ── Frecce TURBINA → INIETTORI ────────────────────────────────────────
        arrow(1.55, 4.37, 1.55, 3.92, color='#336699', lw=2.2,
              lbl=f'{p_man_ox:.0f} bar', lbl_col='#003388')
        arrow(6.45, 4.37, 6.45, 3.92, color='#227755', lw=2.2,
              lbl=f'{p_man_f:.0f} bar', lbl_col='#005533')

        # ── Box INIETTORI ─────────────────────────────────────────────────────
        small_box(1.55, 3.52, 'INJECTOR  OX', dp_inj_ox, p_mcc, '#336699', '#003388')
        small_box(6.45, 3.52, 'INJECTOR FUEL', dp_inj_f,  p_mcc, '#227755', '#005533')

        # ── Frecce INIETTORI → MCC ────────────────────────────────────────────
        arrow(1.90, 3.07, 2.5, 2.90, color='#336699', lw=2.2)
        arrow(6.10, 3.07, 5.5, 2.90, color='#227755', lw=2.2)

        # ── MCC ───────────────────────────────────────────────────────────────
        box(2.0, 1.5, 4.0, 1.7, 'MAIN COMBUSTION CHAMBER',
            [f'P_mcc  {p_mcc:.0f} bar',
             f'T_ad   {CEA_MethaloxCombustion.get_t_ad(max(of_mcc, 0.15), p_mcc):.0f} K',
             f'O/F    {of_mcc:.2f}'],
            p_mcc)

        # ── NOZZLE / THRUST ───────────────────────────────────────────────────
        nozzle_col = _pcol(p_mcc * 0.05, p_max=50)
        noz_patch = mpatches.Polygon(
            [[2.8, 1.5], [5.2, 1.5], [5.7, 0.2], [2.3, 0.2]],
            closed=True,
            facecolor=nozzle_col, edgecolor='#555555', linewidth=1.8,
            zorder=3, alpha=0.85
        )
        ax.add_patch(noz_patch)
        ax.text(4.0, 1.0, f'{curr_thrust:.0f} kN',
                ha='center', va='center', fontsize=11,
                fontweight='bold', color='black', zorder=5)
        ax.text(4.0, 0.4, 'THRUST',
                ha='center', va='center', fontsize=8.5,
                color='black', zorder=5, fontweight='bold')

        # ── Barra colormap pressione ───────────────────────────────────────────
        grad_x = np.linspace(0.3, 7.7, 80)
        dw = grad_x[1] - grad_x[0]
        for i, gx in enumerate(grad_x[:-1]):
            ax.barh(15.0, dw, left=gx, height=0.22,
                    color=_pcol(i / 79 * 800), zorder=2)
        ax.text(0.3, 15.12, '0', ha='center', va='bottom', fontsize=7, color='#333333')
        ax.text(7.7, 15.12, '800 bar', ha='center', va='bottom', fontsize=7, color='#333333')

    # ── Pannello strutturale ──────────────────────────────────────────────────
    def _draw_struct(self, st, engine):
        ax = self.ax_schema
        ax.clear()
        ax.set_facecolor(BG_AX)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis('off')
        ax.set_title('MONITORAGGIO STRUTTURALE — MARGINI DI SICUREZZA',
                     fontsize=11, fontweight='bold', color=COL_TEXT, pad=8)

        # Raccolta dati dal motore
        p_mcc    = st[0] / 1e5
        w_ox     = st[1]
        w_f      = st[2]
        p_orhc   = st[12] / 1e5
        p_frhc   = st[13] / 1e5
        t_orhc   = engine.t_orhc_current
        t_frhc   = engine.t_frhc_current

        # Dati profilo completo dal modello ugello
        sm = self.spatial_model
        p_cool_bar = getattr(engine, 'coolant_pressure_bar', 200.0)
        try:
            i_throat = int(np.argmin(sm.A))
            t_wall_throat_gas  = float(sm.T_hw_rad[i_throat, 0])
            t_wall_throat_cool = float(sm.T_hw_rad[i_throat, -1])
            res_wall = self.struct_analyzer.chamber_wall_profile(
                p_mcc,
                p_cool_bar,
                sm.x,
                sm.r,
                sm.t_hw_profile,
                sm.t_cw,
                sm.T_hw_rad[:, 0],
                sm.T_hw_rad[:, -1],
                sm.M,
                gamma=1.14,
            )
        except Exception:
            t_wall_throat_gas  = 800.0
            t_wall_throat_cool = 300.0
            res_wall = self.struct_analyzer.chamber_wall_profile(
                p_mcc, p_cool_bar,
                sm.x, sm.r, sm.t_hw_profile, sm.t_cw,
                np.full_like(sm.x, 600.0), np.full_like(sm.x, 300.0), sm.M,
            )

        # Calcolo MoS
        results = [
            res_wall,
            self.struct_analyzer.nozzle_throat(t_wall_throat_gas, t_wall_throat_cool),
            self.struct_analyzer.turbine_rotor(w_ox, t_orhc, label='Rotore Turbina OX',  is_ox=True),
            self.struct_analyzer.turbine_rotor(w_f,  t_frhc, label='Rotore Turbina FUEL', is_ox=False),
            self.struct_analyzer.preburner_vessel(p_orhc, label='ORHC — Involucro'),
            self.struct_analyzer.preburner_vessel(p_frhc, label='FRHC — Involucro'),
        ]

        # Layout verticale: 6 righe
        row_h  = 1.45
        y_top  = 9.2
        bar_x0 = 5.2
        bar_w  = 4.2

        for idx, res in enumerate(results):
            y = y_top - idx * row_h
            mos   = res['mos']
            color = StructuralAnalyzer.mos_color(mos)
            status = StructuralAnalyzer.mos_label(mos)

            # Sfondo riga alternato
            bg_col = '#1e2d3d' if idx % 2 == 0 else '#182535'
            ax.add_patch(mpatches.FancyBboxPatch(
                (0.1, y - 0.55), 9.8, row_h - 0.08,
                boxstyle="round,pad=0.05",
                facecolor=bg_col, edgecolor='#2c3e50',
                linewidth=1, zorder=1, alpha=0.9,
            ))

            # Label componente
            ax.text(0.3, y + 0.35, res['label'],
                    ha='left', va='center', fontsize=9, fontweight='bold',
                    color=COL_TEXT, zorder=3)

            # Dettaglio
            ax.text(0.3, y - 0.12, res.get('detail', ''),
                    ha='left', va='center', fontsize=7.5,
                    color='#95a5a6', family='monospace', zorder=3)

            # Barra MoS
            mos_clipped = float(np.clip(mos, -1.0, 3.0))
            bar_frac    = (mos_clipped + 1.0) / 4.0   # -1..3 → 0..1
            bar_filled  = bar_frac * bar_w
            # sfondo barra
            ax.add_patch(mpatches.FancyBboxPatch(
                (bar_x0, y - 0.05), bar_w, 0.32,
                boxstyle="round,pad=0.03",
                facecolor='#2c3e50', edgecolor='#4a6080',
                linewidth=1, zorder=2,
            ))
            # riempimento
            if bar_filled > 0.01:
                ax.add_patch(mpatches.FancyBboxPatch(
                    (bar_x0, y - 0.05), bar_filled, 0.32,
                    boxstyle="round,pad=0.03",
                    facecolor=color, edgecolor='none',
                    linewidth=0, zorder=3, alpha=0.85,
                ))
            # linea zero (MoS=0 → bar_frac=0.25)
            zero_x = bar_x0 + 0.25 * bar_w
            ax.plot([zero_x, zero_x], [y - 0.08, y + 0.30],
                    color='#e74c3c', lw=1.5, zorder=4)

            # Valore MoS e status
            ax.text(bar_x0 + bar_w + 0.15, y + 0.12,
                    f'MoS = {mos:+.2f}',
                    ha='left', va='center', fontsize=9, fontweight='bold',
                    color=color, family='monospace', zorder=4)
            ax.text(bar_x0 + bar_w + 0.15, y - 0.18,
                    status,
                    ha='left', va='center', fontsize=8, fontweight='bold',
                    color=color, zorder=4)

        # Legenda semaforo
        ax.text(0.3, 0.35, '●', color='#2ecc71', fontsize=12, zorder=4)
        ax.text(0.7, 0.35, 'OK (MoS ≥ 0.25)', color='#95a5a6', fontsize=8, va='center', zorder=4)
        ax.text(3.2, 0.35, '●', color='#f39c12', fontsize=12, zorder=4)
        ax.text(3.6, 0.35, 'CAUTION (0 ≤ MoS < 0.25)', color='#95a5a6', fontsize=8, va='center', zorder=4)
        ax.text(7.0, 0.35, '●', color='#e74c3c', fontsize=12, zorder=4)
        ax.text(7.4, 0.35, 'FAILURE', color='#95a5a6', fontsize=8, va='center', zorder=4)

    # ── Loop di simulazione ───────────────────────────────────────────────────
    def _update_sim(self):
        curr_thrust = self.engine.get_current_thrust()
        st          = self.engine.state

        telemetry  = (st[0] / 1e5, st[1], st[2], st[7],
                      self.engine.of_orhc_current, self.engine.of_frhc_current,
                      st[8], st[9], self.engine.mdot_ox_last, self.engine.mdot_f_last)
        cav_status = (self.engine.tp_ox.is_cavitating, self.engine.tp_fuel.is_cavitating)

        th_mov, th_mfv, v_orhc, v_frhc, v_auto_ox, v_auto_f, is_ignited, state_str = \
            self.fc.update(self.target_thrust, curr_thrust, telemetry, cav_status)

        self.engine.cmd_th_mov    = th_mov
        self.engine.cmd_th_mfv    = th_mfv
        self.engine.cmd_v_orhc    = v_orhc
        self.engine.cmd_v_frhc    = v_frhc
        self.engine.cmd_v_auto_ox = v_auto_ox
        self.engine.cmd_v_auto_f  = v_auto_f
        self.engine.is_ignited    = is_ignited

        # Avanzamento del modello matematico (pesante su CPU durante i transitori)
        self.engine.step(self.dt)

        for k in self.hist:
            self.hist[k].pop(0)

        t = self.engine.current_time
        self.hist['t'].append(t)
        self.hist['th'].append(curr_thrust)
        self.hist['th_tgt'].append(self.target_thrust if state_str == "MAIN_STAGE" else 0.0)
        self.hist['p_mcc'].append(st[0] / 1e5)
        self.hist['p_orhc'].append(st[12] / 1e5)
        self.hist['p_frhc'].append(st[13] / 1e5)
        self.hist['of_orhc'].append(self.engine.of_orhc_current)
        self.hist['of_frhc'].append(self.engine.of_frhc_current)
        self.hist['w_ox'].append(st[1])
        self.hist['w_f'].append(st[2])
        self.hist['t_cool'].append(st[7])
        self.hist['p_tank_ox'].append(st[8])
        self.hist['p_tank_f'].append(st[9])
        self.hist['v_mov'].append(st[3] * 100)
        self.hist['v_mfv'].append(st[4] * 100)
        self.hist['v_orhc'].append(st[5] * 100)
        self.hist['v_frhc'].append(st[6] * 100)
        self.hist['v_auto_ox'].append(st[10] * 100)
        self.hist['v_auto_f'].append(st[11] * 100)
        self.hist['mdot_ox'].append(self.engine.mdot_ox_last)
        self.hist['mdot_f'].append(self.engine.mdot_f_last)

        # THROTTLING DELLA GUI: Aggiorniamo Matplotlib solo 1 volta ogni 4 frame
        # per evitare che ax.clear() saturi la coda di Tkinter causando il blocco.
        self._frame_count = getattr(self, '_frame_count', 0) + 1
        
        if self._frame_count % 4 == 0:
            state_colors = {
                "IDLE":       "#4a4a4a",
                "CHILLDOWN":  "#2980b9",
                "SPIN_PRIME": "#e67e22",
                "IGNITION":   "#d35400",
                "RAMP_UP":    "#f39c12",
                "MAIN_STAGE": "#27ae60",
                "MECO":       "#8e44ad",
                "ABORT":      "#c0392b",
            }
            self.lbl_state.config(text=f"  {state_str}  ",
                                  bg=state_colors.get(state_str, "#4a4a4a"))
            self.lbl_abort.config(text=self.fc.abort_reason)

            cool_p_bar = self.engine.coolant_pressure_bar
            phase      = CH4RealGasProps.phase_label(st[7], cool_p_bar)
            mdot_ox    = self.engine.mdot_ox_last
            mdot_f     = self.engine.mdot_f_last
            of_mcc     = mdot_ox / max(mdot_f, 0.01) if mdot_f > 0.01 else 0.0

            self.lbl_tel_perf.config(text=(
                f"THRUST : {curr_thrust:8.1f} kN\n"
                f"P_MCC  : {st[0]/1e5:8.1f} bar\n"
                f"T_COOL : {st[7]:8.1f} K  [{phase}]\n"
                f"P_COOL : {cool_p_bar:8.1f} bar"
            ))
            self.lbl_tel_pb.config(text=(
                f"P_ORHC : {st[12]/1e5:8.1f} bar\n"
                f"T_ORHC : {self.engine.t_orhc_current:8.1f} K\n"
                f"O/F    : {self.engine.of_orhc_current:8.2f}\n"
                f"P_FRHC : {st[13]/1e5:8.1f} bar\n"
                f"T_FRHC : {self.engine.t_frhc_current:8.1f} K\n"
                f"O/F    : {self.engine.of_frhc_current:8.2f}"
            ))
            self.lbl_tel_sys.config(text=(
                f"OX RPM : {st[1]:8.0f}\n"
                f"FL RPM : {st[2]:8.0f}\n"
                f"P_LOX  : {st[8]:8.2f} bar\n"
                f"P_LCH4 : {st[9]:8.2f} bar"
            ))
            self.lbl_tel_mdot.config(text=(
                f"ṁ LOX  : {mdot_ox:8.1f} kg/s\n"
                f"ṁ CH4  : {mdot_f:8.1f} kg/s\n"
                f"O/F MCC: {of_mcc:8.2f}"
            ))
            self.lbl_tel_v.config(text=(
                f"MOV    : {st[3]*100:6.1f} %\n"
                f"MFV    : {st[4]*100:6.1f} %\n"
                f"V_ORHC : {st[5]*100:6.1f} %\n"
                f"V_FRHC : {st[6]*100:6.1f} %\n"
                f"V_A_OX : {st[10]*100:6.1f} %\n"
                f"V_A_FL : {st[11]*100:6.1f} %"
            ))

            T_hist = self.hist['t']
            self.l_th.set_data(T_hist, self.hist['th'])
            self.l_tgt.set_data(T_hist, self.hist['th_tgt'])
            self.l_pmcc.set_data(T_hist, self.hist['p_mcc'])
            self.l_porhc.set_data(T_hist, self.hist['p_orhc'])
            self.l_pfrhc.set_data(T_hist, self.hist['p_frhc'])
            self.l_of_orhc.set_data(T_hist, self.hist['of_orhc'])
            self.l_of_frhc.set_data(T_hist, self.hist['of_frhc'])
            self.l_wox.set_data(T_hist, self.hist['w_ox'])
            self.l_wf.set_data(T_hist, self.hist['w_f'])
            self.l_tcool.set_data(T_hist, self.hist['t_cool'])
            self.l_ptank_ox.set_data(T_hist, self.hist['p_tank_ox'])
            self.l_ptank_f.set_data(T_hist, self.hist['p_tank_f'])
            self.l_v_mov.set_data(T_hist, self.hist['v_mov'])
            self.l_v_mfv.set_data(T_hist, self.hist['v_mfv'])
            self.l_v_auto_ox.set_data(T_hist, self.hist['v_auto_ox'])
            self.l_v_auto_f.set_data(T_hist, self.hist['v_auto_f'])
            self.l_mdot_ox.set_data(T_hist, self.hist['mdot_ox'])
            self.l_mdot_f.set_data(T_hist, self.hist['mdot_f'])

            window = 20.0
            min_x  = max(0.0, t - window)
            max_x  = max(window, t)
            for ax in self._time_axes:
                ax.set_xlim(min_x, max_x)

            if self._right_panel == "SCHEMA":
                self._draw_schema(st, self.engine, state_str)
            elif self._right_panel == "STRUCT":
                self._draw_struct(st, self.engine)
            else:
                P_mcc_pa   = st[0]
                mdot_f_noz = self.engine.mdot_f_last
                mdot_o_noz = self.engine.mdot_ox_last
                if self.engine.is_ignited and mdot_f_noz > 0.01:
                    of_noz     = mdot_o_noz / max(mdot_f_noz, 0.01)
                    T_camera   = CEA_MethaloxCombustion.get_t_ad(of_noz, P_mcc_pa / 1e5)
                    c_star_eff = (CEA_MethaloxCombustion.get_c_star(of_noz, P_mcc_pa / 1e5)
                                  * self.engine.mcc.eta_cstar)
                else:
                    T_camera   = 300.0
                    c_star_eff = 300.0

                cool_p = self.engine.coolant_pressure_bar
                x_vals, tgas_vals, t_hw_hot, t_hw_cold, t_cool_noz, t_cw = \
                    self.spatial_model.compute_instantaneous_profile(
                        P_mcc_pa, T_camera, c_star_eff, mdot_f_noz, 120.0, self.dt,
                        coolant_pressure_bar=cool_p,
                    )
                self.l_gas.set_data(x_vals, tgas_vals)
                self.l_hw_hot.set_data(x_vals, t_hw_hot)
                self.l_hw_cold.set_data(x_vals, t_hw_cold)
                self.l_cool.set_data(x_vals, t_cool_noz)
                self.l_cw.set_data(x_vals, t_cw)

            self.canvas.draw_idle()
            self.root.update_idletasks() # Obbliga Tkinter a processare gli eventi in background sbloccando la GUI

        # Coda la prossima iterazione riducendo il delay per compensare i frame saltati
        self.root.after(10, self._update_sim)


if __name__ == "__main__":
    root = tk.Tk()
    app  = AppGUI(root)
    root.mainloop()

# -*- coding: utf-8 -*-
"""
build_report.py - Dựng 2 bản báo cáo Word từ file gốc của sinh viên + kết quả MATLAB:
  (1) BẢN CHÍNH THỨC  : ghi đè file "Báo cáo_ ... Robot tự hành.docx" (bản gốc được sao lưu)
  (2) BẢN MỞ RỘNG     : "... (BAN MO RONG - giai thich ly thuyet).docx" = bản chính thức + các khung
                        GIẢI THÍCH MỞ RỘNG: suy dẫn chi tiết, nguồn (slide/sách/bài báo - trang, mục,
                        số công thức) và vị trí tương ứng trong mã nguồn.
Số liệu và hình lấy từ MATLAB: ../matlab/results_matlab/results.json, ../matlab/figures_matlab/*.png
Chạy:  python build_report.py
"""
import os, sys, json, shutil, re
import docx
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
from docx.text.paragraph import Paragraph
try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.abspath(os.path.join(HERE, '..'))
ROOT = os.path.abspath(os.path.join(PRJ, '..'))
DOCX = os.path.join(ROOT, 'Báo cáo_ Ứng dụng thuật toán LiDAR Pose-Graph SLAM xây dựng và đánh giá bản đồ cho Robot tự hành.docx')
BACKUP = DOCX.replace('.docx', ' (BAN GOC - backup).docx')
EXTENDED = DOCX.replace('.docx', ' (BAN MO RONG - giai thich ly thuyet).docx')
FIGM = os.path.join(PRJ, 'matlab', 'figures_matlab')
FIGP = os.path.join(PRJ, 'figures')
R = json.load(open(os.path.join(PRJ, 'matlab', 'results_matlab', 'results.json'), encoding='utf-8'))
PYRES = os.path.join(PRJ, 'results', 'results.json')
PY = json.load(open(PYRES, encoding='utf-8')) if os.path.exists(PYRES) else None
MAIN_M = os.path.join(PRJ, 'matlab', 'main_pose_graph_slam.m')
S1, MC, SE, TB, CFG = R['scenario1'], R['monte_carlo'], R['sensitivity'], R.get('lidarslam', {}), R['cfg']
if isinstance(SE, dict):
    SE = [SE]
dr, nl, sl, info = S1['dr'], S1['nolc'], S1['slam'], S1['info']
if not os.path.exists(BACKUP):
    shutil.copy2(DOCX, BACKUP); print('Đã sao lưu bản gốc ->', BACKUP)
ORIG = [p.text for p in docx.Document(BACKUP).paragraphs]


def orig(prefix):
    for t in ORIG:
        if t.strip().startswith(prefix):
            return t.strip()
    raise KeyError(prefix)


def fig_path(name):
    for d_ in (FIGM, FIGP):
        if os.path.exists(os.path.join(d_, name)):
            return os.path.join(d_, name)
    return None


f1 = lambda x: f'{x:.1f}'; f2 = lambda x: f'{x:.2f}'; f3 = lambda x: f'{x:.3f}'
pct = lambda a, b: 100 * (1 - b / a)


def build(EXT, out_path):
    d = docx.Document(BACKUP)
    body = d.element.body
    keep_until = next(p._p for p in d.paragraphs if p.text.startswith('Thời gian thực hiện'))
    started = False
    for el in list(body):
        if started and el.tag != qn('w:sectPr'):
            body.remove(el)
        if el is keep_until:
            started = True
    if EXT:   # ghi chú trên bìa
        p = d.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run('BẢN MỞ RỘNG – kèm giải thích chi tiết lý thuyết và nguồn trích dẫn (tài liệu học tập, không thay thế bản nộp chính thức)')
        r.italic = True; r.bold = True; r.font.color.rgb = RGBColor(0xC0, 0x30, 0x30)
    sec = d.sections[0]
    sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    sec.left_margin, sec.right_margin, sec.top_margin, sec.bottom_margin = Cm(3.0), Cm(2.0), Cm(2.0), Cm(2.0)

    def set_font(style, size, bold=None, italic=None):
        style.font.name = 'Times New Roman'; style.font.size = Pt(size)
        rpr = style.element.get_or_add_rPr(); rf = rpr.find(qn('w:rFonts'))
        if rf is None:
            rf = OxmlElement('w:rFonts'); rpr.append(rf)
        for a in ('w:ascii', 'w:hAnsi', 'w:cs', 'w:eastAsia'):
            rf.set(qn(a), 'Times New Roman')
        if bold is not None: style.font.bold = bold
        if italic is not None: style.font.italic = italic
        style.font.color.rgb = RGBColor(0, 0, 0)

    for st in d.styles:
        if st.name in ('Normal', 'normal'):
            set_font(st, 13); pf = st.paragraph_format
            pf.line_spacing = 1.4; pf.space_after = Pt(4); pf.space_before = Pt(0); pf.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    for nm, size in [('Heading 1', 15), ('Heading 2', 13.5), ('Heading 3', 13)]:
        st = d.styles[nm]; set_font(st, size, bold=True, italic=(nm == 'Heading 3'))
        pf = st.paragraph_format; pf.space_before = Pt(12 if nm == 'Heading 1' else 8); pf.space_after = Pt(6)
        pf.keep_with_next = True; pf.line_spacing = 1.2; pf.alignment = WD_ALIGN_PARAGRAPH.LEFT
    set_font(d.styles['Title'], 18, bold=True)
    for p in d.paragraphs:
        for r in p.runs:
            r.font.name = 'Times New Roman'; r._element.get_or_add_rPr().get_or_add_rFonts().set(qn('w:eastAsia'), 'Times New Roman')

    FIGN, TABN, EQN, EXTN = {}, {}, [0], [0]

    def add_rich(p, text, bold=False, italic=False, size=None, color=None):
        for part in re.split(r'(\*\*[^*]+\*\*)', text):
            if not part:
                continue
            b = part.startswith('**') and part.endswith('**')
            r = p.add_run(part[2:-2] if b else part); r.bold = bold or b; r.italic = italic
            if size: r.font.size = Pt(size)
            if color: r.font.color.rgb = color
        return p

    def para(text='', align=None, bold=False, italic=False, size=None, indent=True, container=None):
        p = (container or d).add_paragraph()
        if text: add_rich(p, text, bold=bold, italic=italic, size=size)
        if align is not None: p.alignment = align
        if indent and align is None: p.paragraph_format.first_line_indent = Cm(0.75)
        return p

    def h1(t):
        p = d.add_paragraph(style='Heading 1'); p.add_run(t); p.paragraph_format.page_break_before = True
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER

    def h2(t): d.add_paragraph(t, style='Heading 2')
    def h3(t): d.add_paragraph(t, style='Heading 3')

    def bullet(text, container=None, size=None):
        p = (container or d).add_paragraph()
        p.paragraph_format.left_indent = Cm(0.9); p.paragraph_format.first_line_indent = Cm(-0.45)
        add_rich(p, '•  ' + text, size=size); return p

    def eq(text, chap, container=None, number=True):
        p = (container or d).add_paragraph()
        p.paragraph_format.tab_stops.add_tab_stop(Cm(8.0), alignment=1); p.paragraph_format.tab_stops.add_tab_stop(Cm(16.0), alignment=2)
        p.paragraph_format.first_line_indent = Cm(0); p.alignment = WD_ALIGN_PARAGRAPH.LEFT
        r = p.add_run('\t' + text); r.font.name = 'Cambria Math'
        r._element.get_or_add_rPr().get_or_add_rFonts().set(qn('w:hAnsi'), 'Cambria Math')
        if not number:
            return ''
        EQN[0] += 1; p.add_run(f'\t({chap}.{EQN[0]})'); return f'({chap}.{EQN[0]})'

    def figure(name, key, caption, width=15.5, note=None):
        path = fig_path(name)
        if path is None:
            print('  ! thiếu hình', name); return
        FIGN[key] = caption
        p = d.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.keep_with_next = True
        p.paragraph_format.first_line_indent = Cm(0); p.add_run().add_picture(path, width=Cm(width))
        c = d.add_paragraph(); c.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = c.add_run(f'Hình {key}: '); r.bold = True; r.font.size = Pt(12)
        r = c.add_run(caption + f' (Nguồn: {note or "nhóm tự thực hiện"})'); r.italic = True; r.font.size = Pt(12)

    def borders(t):
        tblPr = t._tbl.tblPr; bd = OxmlElement('w:tblBorders')
        for side in ('top', 'left', 'bottom', 'right', 'insideH', 'insideV'):
            e = OxmlElement(f'w:{side}'); e.set(qn('w:val'), 'single'); e.set(qn('w:sz'), '4'); e.set(qn('w:space'), '0'); e.set(qn('w:color'), '000000'); bd.append(e)
        look = tblPr.find(qn('w:tblLook')); (look.addprevious(bd) if look is not None else tblPr.append(bd))

    def shade(cell, color):
        tcPr = cell._element.get_or_add_tcPr(); sh = OxmlElement('w:shd')
        sh.set(qn('w:val'), 'clear'); sh.set(qn('w:color'), 'auto'); sh.set(qn('w:fill'), color); tcPr.append(sh)

    def table(key, caption, header, rows, widths=None, size=11):
        if key: TABN[key] = caption
        if caption:
            c = d.add_paragraph(); c.alignment = WD_ALIGN_PARAGRAPH.CENTER; c.paragraph_format.keep_with_next = True
            if key:
                r = c.add_run(f'Bảng {key}: '); r.bold = True; r.font.size = Pt(12)
            r = c.add_run(caption); r.italic = True; r.font.size = Pt(12)
        t = d.add_table(rows=1 + len(rows), cols=len(header)); t.alignment = WD_TABLE_ALIGNMENT.CENTER; borders(t)
        for i, row in enumerate([header] + rows):
            for j, v in enumerate(row):
                cell = t.cell(i, j); cell.text = ''; pp = cell.paragraphs[0]
                pp.paragraph_format.first_line_indent = Cm(0); pp.paragraph_format.line_spacing = 1.1; pp.paragraph_format.space_after = Pt(1)
                pp.alignment = WD_ALIGN_PARAGRAPH.CENTER if (i == 0 or j > 0) else WD_ALIGN_PARAGRAPH.LEFT
                add_rich(pp, str(v), bold=(i == 0), size=size)
                if i == 0: shade(cell, 'D9E2F3')
                if widths: cell.width = Cm(widths[j])
        d.add_paragraph().paragraph_format.space_after = Pt(2)

    def ext(title, items):
        """Khung GIẢI THÍCH MỞ RỘNG (chỉ có trong bản mở rộng).
        items: ('p', text) | ('eq', text) | ('b', text) | ('src', text) | ('code', text)"""
        if not EXT:
            return
        EXTN[0] += 1
        t = d.add_table(rows=1, cols=1); t.alignment = WD_TABLE_ALIGNMENT.CENTER; borders(t)
        cell = t.cell(0, 0); shade(cell, 'FFF7E0'); cell.width = Cm(16)
        p0 = cell.paragraphs[0]; p0.paragraph_format.first_line_indent = Cm(0)
        add_rich(p0, f'GIẢI THÍCH MỞ RỘNG {EXTN[0]} – {title}', bold=True, size=12, color=RGBColor(0x9C, 0x4A, 0x00))
        for kind, txt in items:
            if kind == 'eq':
                eq(txt, 0, container=cell, number=False)
            elif kind == 'b':
                bullet(txt, container=cell, size=12)
            elif kind == 'src':
                p = cell.add_paragraph(); p.paragraph_format.first_line_indent = Cm(0)
                add_rich(p, 'Nguồn: ', bold=True, size=11.5, color=RGBColor(0x1F, 0x4E, 0x99)); add_rich(p, txt, italic=True, size=11.5)
            elif kind == 'code':
                p = cell.add_paragraph(); p.paragraph_format.first_line_indent = Cm(0)
                add_rich(p, 'Trong mã nguồn: ', bold=True, size=11.5, color=RGBColor(0x2E, 0x7D, 0x32)); add_rich(p, txt, size=11.5)
            else:
                p = cell.add_paragraph(); p.paragraph_format.first_line_indent = Cm(0); add_rich(p, txt, size=12)
        d.add_paragraph().paragraph_format.space_after = Pt(2)

    def page_break():
        d.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

    def field(p, instr, placeholder=''):
        for kind, val in (('begin', None), ('instr', instr), ('separate', None), ('text', placeholder), ('end', None)):
            if kind == 'text':
                p.add_run(val); continue
            r = p.add_run()
            if kind == 'instr':
                it = OxmlElement('w:instrText'); it.set(qn('xml:space'), 'preserve'); it.text = val; r._r.append(it)
            else:
                fc = OxmlElement('w:fldChar'); fc.set(qn('w:fldCharType'), kind); r._r.append(fc)

    # ======================================================== PHẦN ĐẦU
    page_break()
    para('MỤC LỤC', align=WD_ALIGN_PARAGRAPH.CENTER, bold=True, size=15)
    field(d.add_paragraph(), 'TOC \\o "1-3" \\h \\z \\u', '[Mở file trong Word → chuột phải vào đây → Update Field → Update entire table]')
    page_break()
    para('DANH MỤC HÌNH VẼ', align=WD_ALIGN_PARAGRAPH.CENTER, bold=True, size=15); FIG_ANCHOR = d.add_paragraph()
    para('DANH MỤC BẢNG BIỂU', align=WD_ALIGN_PARAGRAPH.CENTER, bold=True, size=15); TAB_ANCHOR = d.add_paragraph()
    para('DANH MỤC TỪ VIẾT TẮT', align=WD_ALIGN_PARAGRAPH.CENTER, bold=True, size=15)
    abbr = [('AGV / AMR', 'Xe dẫn đường tự động / Robot di động tự hành'), ('ANEES', 'NEES trung bình (Average NEES)'),
            ('ATE / RMSE', 'Sai số quỹ đạo tuyệt đối / Sai số căn quân phương'), ('DR', 'Định vị suy đoán bằng odometry (Dead Reckoning)'),
            ('EKF', 'Bộ lọc Kalman mở rộng (Extended Kalman Filter)'), ('GN', 'Phương pháp Gauss–Newton'),
            ('ICP', 'So khớp điểm lặp gần nhất (Iterative Closest Point)'), ('IoU', 'Tỷ lệ giao trên hợp (Intersection-over-Union)'),
            ('LC', 'Đóng vòng (Loop Closure)'), ('LiDAR', 'Cảm biến đo khoảng cách bằng laser'), ('MAP', 'Hậu nghiệm cực đại (Maximum A Posteriori)'),
            ('NEES', 'Bình phương sai số ước lượng chuẩn hóa (Normalized Estimation Error Squared)'), ('RPE', 'Sai số tư thế tương đối (Relative Pose Error)'),
            ('RVC3', 'Robotics, Vision & Control Toolbox (sách Corke et al., 2023)'), ('SE(2)', 'Nhóm phép biến đổi cứng trong mặt phẳng'),
            ('SLAM', 'Định vị và lập bản đồ đồng thời (Simultaneous Localization and Mapping)'), ('SPA', 'Điều chỉnh tư thế thưa (Sparse Pose Adjustment)')]
    table(None, None, ['Viết tắt', 'Ý nghĩa'], [[a, b] for a, b in abbr], widths=[3.2, 12.8])
    if EXT:
        para('HƯỚNG DẪN ĐỌC BẢN MỞ RỘNG', align=WD_ALIGN_PARAGRAPH.CENTER, bold=True, size=14)
        para('Bản này giữ nguyên toàn bộ nội dung bản chính thức và chèn thêm các khung nền vàng “GIẢI THÍCH MỞ RỘNG”. Mỗi khung gồm: (i) giải thích trực giác, '
             '(ii) suy dẫn toán học từng bước, (iii) **Nguồn** – vị trí chính xác trong slide học phần, sách Corke et al. (2023) hoặc bài báo (trang, mục, số công thức), '
             '(iv) **Trong mã nguồn** – hàm tương ứng trong main_pose_graph_slam.m. Bảng tổng hợp “lý thuyết → nguồn → mã” ở Phụ lục C. '
             'Quy ước trang sách: số trang in trên sách (bản PDF lệch +23 trang).')

    # ======================================================== CHƯƠNG 1
    EQN[0] = 0
    h1('CHƯƠNG 1. GIỚI THIỆU')
    h2('1.1. Giới thiệu chủ đề và bối cảnh ứng dụng thực tiễn')
    para(orig('Bối cảnh và môi trường hoạt động'))
    para('**Đối tượng robot và nhiệm vụ:** robot AMR có động học kiểu xe đạp (bicycle model), trang bị encoder (odometry) và LiDAR 2D quét 360°, '
         'chạy một vòng tuần tra khép kín qua các hành lang của phân xưởng 20 m × 20 m để đồng thời tự định vị và dựng bản đồ lưới chiếm chỗ phục vụ điều hướng.')
    para('**Vấn đề kỹ thuật cần giải quyết:**', indent=False)
    bullet(orig('Hiện tượng tích lũy sai số trôi'))
    bullet('Biểu diễn và đánh giá độ bất định: theo Chương 4 của học phần, mọi ước lượng tư thế phải đi kèm hiệp phương sai; với dead reckoning độ bất định tăng '
           'không giới hạn và sai số chỉ đáng tin khi nằm trong biên tin cậy 95 % (±2σ). Cần kiểm chứng pose-graph SLAM có thu hẹp được các biên này và các biên '
           'có phản ánh đúng sai số thật (tính nhất quán) hay không.')
    bullet(orig('Bài toán kiểm chứng định lượng'))
    para('**Giải pháp và ý nghĩa:** quỹ đạo thật của robot được sinh bằng mô hình Simulink sl_drivepoint của sách Corke et al. (2023); đề tài tự cài đặt '
         'LiDAR Pose-Graph SLAM trên MATLAB (front-end ICP điểm–đường + phát hiện đóng vòng; back-end Gauss–Newton theo Grisetti et al., 2010), từ đó vừa có quỹ đạo tối ưu, '
         'vừa có ma trận hiệp phương sai Σ = H⁻¹ để vẽ hai đường biên 95 % và elip 95 %; so sánh với dead reckoning (như slide), trường hợp không đóng vòng và đối tượng '
         'lidarSLAM của Navigation Toolbox. Một demo 3D (Python) phát lại kết quả để trực quan hóa robot hoạt động trong nhà xưởng.')
    figure('fig1_moi_truong_lidar.png', '1.1', 'Bài toán khảo sát: mặt bằng nhà xưởng chuẩn, quỹ đạo Simulink của AMR và mô phỏng chùm tia LiDAR', 16)
    h2('1.2. Mục tiêu của đề tài')
    para('**Mục tiêu chung:** nghiên cứu cơ sở lý thuyết, xây dựng mô phỏng có đối chứng Ground Truth trên MATLAB/Simulink và đánh giá định lượng cả độ chính xác '
         'lẫn độ tin cậy (tính nhất quán thống kê) của LiDAR Pose-Graph SLAM cho AMR trong nhà xưởng.')
    para('**Các mục tiêu cụ thể (có thể kiểm chứng):**', indent=False)
    for t in ['MT1 – Hệ thống hóa lý thuyết: dead reckoning và lan truyền hiệp phương sai (slide Ch.4), bản đồ lưới chiếm chỗ (Elfes, 1989), so khớp quét và đóng vòng '
              '(Hess et al., 2016), tối ưu đồ thị tư thế (Grisetti et al., 2010), thước đo sai số SLAM (Kümmerle et al., 2009).',
              'MT2 – Thiết kế mặt bằng 20 m × 20 m (ô 5 cm) làm Ground Truth; sinh quỹ đạo bằng Simulink sl_drivepoint; mô phỏng odometry và LiDAR có nhiễu.',
              'MT3 – Tự cài đặt front-end (ICP điểm–đường, phát hiện đóng vòng) và back-end (Gauss–Newton trên ma trận thông tin thưa H), tính Σ = H⁻¹.',
              'MT4 – So sánh (A) dead reckoning, (B) không đóng vòng, (C) pose-graph SLAM theo ATE, RPE, Map Accuracy, Obstacle IoU; vẽ sai số x, y, θ cùng hai '
              'đường biên ±2σ (95 %) và elip 95 %; đối chiếu lidarSLAM.',
              'MT5 – Kiểm định nhất quán bằng Monte Carlo và khảo sát độ nhạy theo nhiễu LiDAR σ_r = 1–8 cm; xây dựng demo 3D.']:
        bullet(t)
    para('**Tiêu chí xác nhận:** ATE RMSE của pose-graph SLAM < 0.15 m ở σ_r = 3 cm; Map Accuracy > 90 %; ≥ 90 % mẫu sai số nằm trong biên ±2σ.')
    h2('1.3. Phạm vi thực hiện và giới hạn')
    para(orig('Phạm vi nghiên cứu'))
    para('**Công cụ:** MATLAB R2024b + Simulink + RVC3 Toolbox của sách (mô hình sl_drivepoint, hàm plotvehicle) – sinh toàn bộ số liệu và hình của báo cáo; '
         'Navigation Toolbox (lidarSLAM) để đối chiếu; Python 3 + PyVista cho demo 3D phát lại kết quả. Không triển khai trên robot thật; không xét vật thể động và sai số hệ thống của odometry.')
    h2('1.4. Cấu trúc của báo cáo')
    for t in ['Chương 1: bối cảnh, vấn đề kỹ thuật, mục tiêu, phạm vi.',
              'Chương 2: dead reckoning và biên tin cậy 95 %, bản đồ lưới, front-end LiDAR, tối ưu đồ thị tư thế và hiệp phương sai, phương pháp đánh giá, so sánh phương pháp.',
              'Chương 3: phát biểu bài toán, phương pháp, mô hình Simulink, thiết kế thuật toán, thông số và kịch bản.',
              'Chương 4: kết quả 4 kịch bản, demo 3D, đối chiếu mục tiêu, hạn chế.', 'Chương 5: kết luận và hướng phát triển.']:
        bullet(t)

    # ======================================================== CHƯƠNG 2
    EQN[0] = 0
    h1('CHƯƠNG 2. CƠ SỞ LÝ THUYẾT')
    h2('2.1. Tổng quan về bài toán SLAM và các hướng tiếp cận')
    for pre in ['Trong các ứng dụng vận chuyển', 'Theo Grisetti et al. (2010), xét trên']:
        para(orig(pre))
    eq('p(x₁:T , m | z₁:T , u₁:T , x₀)', 2)
    para(orig('Dựa trên cấu trúc mạng Bayes động'))
    bullet(orig('Phương pháp lọc (Filtering')); bullet(orig('Phương pháp làm trơn dựa trên đồ thị'))
    ext('Vì sao SLAM là bài toán ước lượng xác suất', [
        ('p', 'Slide Ch.4 part 2 định nghĩa định vị là bài toán ước lượng: “suy ra giá trị của đại lượng x bằng cách xử lý dữ liệu phụ thuộc vào x”, và mô tả vị trí robot '
              'bằng một hàm mật độ xác suất (PDF) thay vì một điểm. SLAM mở rộng điều đó cho cả quỹ đạo lẫn bản đồ: ta muốn phân bố hậu nghiệm (2.1).'),
        ('p', 'Theo quy tắc Bayes và giả thiết Markov, hậu nghiệm phân rã thành tích của các mô hình chuyển động p(x_t | x_{t−1}, u_t) và mô hình quan sát p(z_t | x_t, m). '
              'Lấy −log của tích các hàm Gauss sẽ ra một **tổng bình phương có trọng số** – đây chính là lý do graph-SLAM quy về bài toán bình phương tối thiểu (mục 2.5).'),
        ('eq', '−log p(x | z) = ½ Σ_{<i,j>} e_ijᵀ Ω_ij e_ij + const   ⇒   x_MAP = argmin_x Σ e_ijᵀ Ω_ij e_ij'),
        ('p', 'Lọc (EKF) chỉ giữ phân bố của tư thế hiện tại (marginalize quá khứ), còn làm trơn (graph) giữ toàn bộ quỹ đạo nên có thể “sửa quá khứ” khi đóng vòng.'),
        ('src', 'Slide Ch.4 part 2, tr. 7–9 (định nghĩa bài toán định vị, PDF); Grisetti et al. (2010), mục II–III và công thức (1); Corke et al. (2023), mục 6.4, tr. 257.')])

    h2('2.2. Định vị bằng dead reckoning và biểu diễn độ bất định (theo slide Chương 4)')
    para('Phần này trình bày đúng mạch kiến thức của slide Chương 4 – Part 2 (mục 4.5.2) và Corke et al. (2023, mục 6.1), làm nền cho việc vẽ hai đường biên 95 % ở Chương 4.')
    h3('2.2.1. Mô hình chuyển động theo odometry')
    para('Gọi cấu hình robot là x = (x, y, θ). Theo slide (4.5.2.1), phép đo odometry trong khoảng thứ k là δ_k = (δd, δθ): robot tiến δd theo trục x thân xe rồi quay δθ:')
    EQ_MOT = eq('x_{k+1} = x_k + δd cos θ_k ,   y_{k+1} = y_k + δd sin θ_k ,   θ_{k+1} = θ_k + δθ', 2)
    para('Trong đề tài, robot chạy theo đường cong (quỹ đạo Simulink), nên odometry giữa hai keyframe được biểu diễn tổng quát bằng tư thế tương đối trong khung thân xe '
         'z_k = (Δx, Δy, Δθ) và x_{k+1} = x_k ⊕ (z_k + v_k), với nhiễu v_k ~ N(0, V), V = diag(σ_d², σ_ngang², σ_θ²); mô hình của slide là trường hợp riêng Δy = 0. '
         f'Đề tài dùng σ_d = {CFG["sigma_d"]} m và σ_θ = {CFG["sigma_th_deg"]:.1f}° như ví dụ trong sách, σ_ngang = {CFG["sigma_lat"]} m (trượt bánh nhỏ).')
    ext('Phép hợp thành ⊕ và mô hình odometry tổng quát', [
        ('p', 'Toán tử ⊕ (compounding) ghép tư thế b (biểu diễn trong khung a) vào tư thế a:'),
        ('eq', 'a ⊕ b = [ a_x + b_x cos a_θ − b_y sin a_θ ;  a_y + b_x sin a_θ + b_y cos a_θ ;  a_θ + b_θ ]'),
        ('p', 'Với b = (δd, 0, δθ) ta thu lại đúng (2.2). Toán tử ngược ⊖ cho tư thế tương đối: a ⊖ b = R(a_θ)ᵀ(t_b − t_a), θ_b − θ_a. Odometry “thật” trong mô phỏng là '
              'z_k = x_{k−1} ⊖ x_k (từ quỹ đạo Simulink), odometry đo được là z_k + v_k.'),
        ('src', 'Slide Ch.4 part 2, tr. 11–13 (Modeling the robot, odometry error); Corke et al. (2023), mục 6.1.1, công thức (6.1)–(6.2), tr. 219; ⊕/⊖: Kümmerle et al. (2009), mục 3.'),
        ('code', 'hàm oplus(), relpose(), deadReckoning() và đoạn “odo = zt + randn(N−1,3)*sqrt(V)” trong runSLAM().')])
    h3('2.2.2. Lan truyền hiệp phương sai (bước dự đoán của EKF)')
    para('Ước lượng x̂ và hiệp phương sai P được cập nhật đệ quy (slide 4.5.2.2; Corke, công thức 6.3–6.4):')
    EQ_EKF = eq('x̂_{k+1} = x̂_k ⊕ ẑ_k ,     P_{k+1} = F_x P_k F_xᵀ + F_v V F_vᵀ', 2)
    EQ_JAC = eq('F_x = [1  0  −Δx sinθ − Δy cosθ ; 0  1  Δx cosθ − Δy sinθ ; 0  0  1] ,   F_v = [R(θ)  0 ; 0  1]', 2)
    para('Vì F_v V F_vᵀ luôn xác định dương nên P không bao giờ giảm: độ bất định của dead reckoning tăng không giới hạn (slide 4.5.2.2). Các phần tử ngoài đường chéo '
         'thể hiện tương quan, ví dụ sai số góc θ sinh ra sai số vị trí.')
    ext('Suy dẫn Jacobian và công thức lan truyền hiệp phương sai', [
        ('p', 'Bước 1 – Tuyến tính hóa: với y = f(x) và x ~ N(μ, Σ_x), khai triển Taylor bậc 1: y ≈ f(μ) + F(x − μ), F = ∂f/∂x. Khi đó E[y] ≈ f(μ) và'),
        ('eq', 'Σ_y = E[(y − ȳ)(y − ȳ)ᵀ] ≈ F E[(x − μ)(x − μ)ᵀ] Fᵀ = F Σ_x Fᵀ   (slide 4.5.2.2, tr. 17)'),
        ('p', 'Bước 2 – Với f(x, v) = x ⊕ (z + v) có hai nguồn bất định độc lập (x̂_k và nhiễu v), ta cộng hai số hạng: P_{k+1} = F_x P_k F_xᵀ + F_v V F_vᵀ.'),
        ('p', 'Bước 3 – Đạo hàm: x_{k+1} = x + Δx cosθ − Δy sinθ ⇒ ∂/∂θ = −Δx sinθ − Δy cosθ; y_{k+1} = y + Δx sinθ + Δy cosθ ⇒ ∂/∂θ = Δx cosθ − Δy sinθ; ∂/∂x = ∂/∂y = 1. '
              'Theo nhiễu: ∂(x_{k+1}, y_{k+1})/∂(v_x, v_y) = R(θ), ∂θ_{k+1}/∂v_θ = 1 ⇒ F_v = blkdiag(R(θ), 1). Với Δy = 0 ta được đúng F_x, F_v trên slide.'),
        ('p', 'Bước 4 – Vì sao P “chỉ tăng”: det F_x = 1 nên det(F_x P F_xᵀ) = det P; cộng thêm ma trận nửa xác định dương F_v V F_vᵀ thì det P_{k+1} ≥ det P_k. '
              'Slide (tr. 21) và sách (tr. 225) vẽ √det P theo thời gian để minh họa điều này.'),
        ('src', 'Slide Ch.4 part 2, tr. 14–17, 21; Corke et al. (2023), công thức (6.3)–(6.7), tr. 222–223 và tr. 225; Phụ lục E (Linearization, Jacobians) tr. 737–738.'),
        ('code', 'deadReckoning(): “Fx = [1 0 -s*z(1)-c*z(2); 0 1 c*z(1)-s*z(2); 0 0 1]; Fv = [c -s 0; s c 0; 0 0 1]; Pk(:,:,k+1) = Fx*Pk*Fx'' + Fv*V*Fv''”.')])
    h3('2.2.3. Khoảng tin cậy 95 % (hai đường biên ±2σ) và elip sai số')
    para('Với phân bố chuẩn một chiều, xác suất để sai số nằm trong [−2σ, +2σ] là 95.4 % (chính xác 95 % ứng với ±1.96σ). Slide 4.5.2.2 phát biểu “có 95 % khả năng tọa độ x '
         'nằm trong biên ±2σ”. Vì vậy với σ_x,k = √P_k(1,1) ta vẽ sai số x̃_k = x̂_k − x_k cùng **hai đường biên +2σ_x,k và −2σ_x,k** theo bước k (tương tự cho y, θ). Ước lượng '
         'nhất quán khi khoảng 95 % mẫu nằm giữa hai biên; vượt biên thường xuyên là “quá tự tin” (overconfident), luôn rất nhỏ so với biên là “quá thận trọng”.')
    para('Trong hai chiều, elip tin cậy 95 % (Excurse: Error Ellipse) gồm các điểm p thỏa:')
    EQ_ELL = eq('(p − μ)ᵀ P_xy⁻¹ (p − μ) ≤ χ²₂(0.95) = 5.991', 2)
    para('với P_xy là khối 2×2 vị trí của P; bán trục elip là √(5.991 λ_i) theo các vector riêng của P_xy. Vế trái chính là NEES vị trí (mục 2.6.3).')
    ext('Vì sao ±2σ ↔ 95 % và vì sao hằng số elip là 5.991', [
        ('p', 'Một chiều: nếu e ~ N(0, σ²) thì P(|e| ≤ kσ) = 2Φ(k) − 1. Với k = 2: 2·0.97725 − 1 = 0.9545; với k = 1.96: 0.95; với k = 3: 0.9973.'),
        ('p', 'Hai chiều: đặt u = L⁻¹(p − μ) với P_xy = L Lᵀ (Cholesky) thì u ~ N(0, I₂), nên d² = (p − μ)ᵀ P_xy⁻¹ (p − μ) = u₁² + u₂² ~ χ²₂. Hàm phân phối của χ²₂ có dạng đóng '
              'F(d²) = 1 − exp(−d²/2), do đó mức 95 % cho d² = −2 ln(0.05) = 5.991 (hệ số bán trục √5.991 = 2.448).'),
        ('eq', '1 − exp(−d²/2) = 0.95   ⇔   d² = −2 ln 0.05 = 5.991'),
        ('p', 'Lưu ý quy ước: hàm EKF.ploterror của toolbox RVC3 vẽ biên √χ²₂(0.95)·σ ≈ ±2.45σ cho từng trục, còn slide phát biểu ±2σ. Báo cáo dùng ±2σ cho đồ thị từng trục '
              '(đúng slide, 95.4 %) và 5.991 cho elip 2D (đúng định nghĩa elip 95 %). Elip “1σ” trên slide tương ứng d² = 1 (chỉ chứa 39.3 % xác suất trong 2D).'),
        ('p', 'Diễn giải theo slide Ch.4 part 1 (ví dụ đo chiều cao tòa nhà): nếu nhiều mẫu nằm ngoài vùng 95 % thì bộ lọc “quá tự tin” (overconfident) – phải tăng nhiễu đo/nhiễu quá trình.'),
        ('src', 'Slide Ch.4 part 1, tr. 14–15 (confidence interval, covariance ellipse), tr. 96–100 (95 % confidence, overconfident); Slide Ch.4 part 2, tr. 19 (±2σ), tr. 23 (Error Ellipse); '
                'Corke et al. (2023), tr. 224–225 (plotellipse, chi2inv(C,2)) và Phụ lục G (Gaussian Random Variables); toolbox RVC3: EKF.m, hàm ploterror (dòng 425).'),
        ('code', 'ellipse95() (bán trục √(5.991λ)), boundPlot() (vẽ 2 đường ±2σ), metrics() (tính % mẫu trong biên, NEES).')])
    h3('2.2.4. Từ quan sát mốc (EKF-SLAM) đến đồ thị tư thế')
    para('Slide 4.5.3 chỉ ra rằng chỉ có thông tin ngoại cảm (quan sát landmark) mới làm elip co lại: khi robot quan sát lại một mốc đã biết, hiệp phương sai giảm mạnh và sai số chủ '
         'yếu nằm trong biên 95 %. EKF-SLAM có hạn chế: ma trận tăng theo số mốc (bộ nhớ O(M²)), giả thiết Gauss và tuyến tính hóa một lần (slide 4.5.3.3). Pose-graph SLAM '
         '(Corke et al., 2023, mục 6.4) thay việc lọc bằng tối ưu toàn cục trên đồ thị; khi robot quay lại nơi cũ, cạnh đóng vòng đóng vai trò như quan sát lại landmark, '
         '“kéo” cả quỹ đạo về đúng và thu nhỏ elip (Hình 2.1).')
    figure('fig_khai_niem_pose_graph.png', '2.1', 'Nguyên lý pose-graph SLAM: đóng vòng phân bổ sai số tích lũy lên toàn đồ thị', 15,
           'nhóm vẽ lại theo ý tưởng Hình 6.29 của Corke et al., 2023')
    ext('Vì sao quan sát làm elip co lại (bước cập nhật EKF)', [
        ('p', 'Bước cập nhật EKF (slide 4.5.3.1, tr. 28–31): K = P Hᵀ (H P Hᵀ + W)⁻¹, x̂ ← x̂ + K ν, P ← (I − K H) P. Trong dạng thông tin: P⁺⁻¹ = P⁻¹ + Hᵀ W⁻¹ H – thông tin chỉ được '
              '**cộng thêm**, nên hiệp phương sai giảm. Pose-graph làm đúng điều này ở dạng thông tin: mỗi cạnh cộng Jᵀ Ω J vào H (mục 2.5.2); cạnh đóng vòng cộng thông tin nối hai nút '
              'xa nhau trong thời gian nên giảm bất định của cả đoạn quỹ đạo giữa chúng.'),
        ('src', 'Slide Ch.4 part 2, tr. 25–33 (localizing in a landmark map), tr. 45–52 (EKF-SLAM và hạn chế); Corke et al. (2023), mục 6.2 và 6.4, Hình 6.29 (tr. 257).')])

    h2('2.3. Biểu diễn môi trường bằng bản đồ lưới chiếm chỗ (Occupancy Grid Map)')
    para(orig('Để biểu diễn không gian làm việc'))
    h3('2.3.1. Mô hình trường ngẫu nhiên và quy tắc cập nhật Bayes')
    para(orig('Theo Elfes (1989), bản đồ lưới chiếm chỗ chia'))
    eq('P[s(C_i) = OCC] + P[s(C_i) = EMP] = 1', 2)
    para(orig('Khi mô hình hóa bản đồ lưới như một trường ngẫu nhiên'))
    eq('P[s(C_i)=OCC | {r}_{t+1}] = p[r_{t+1}|OCC]·P[OCC|{r}_t] / Σ_{s} p[r_{t+1}|s]·P[s|{r}_t]', 2)
    para(orig('Trong công thức trên, ước lượng ở bước trước'))
    para('Trong cài đặt, đề tài dùng dạng log-odds l = log[P/(1 − P)]: ô “trúng” (hit) cộng l_hit = log(0.7/0.3), ô tia đi xuyên qua (miss) cộng l_miss = log(0.4/0.6), '
         'giới hạn l ∈ [−4, 4] (tương đương hàm clamp của Hess et al., 2016).')
    EQ_LO = eq('l_t(C_i) = l_{t−1}(C_i) + log[ p(OCC | r_t) / (1 − p(OCC | r_t)) ]', 2)
    ext('Từ công thức Bayes của Elfes đến phép cộng log-odds', [
        ('p', 'Viết công thức Bayes cho OCC và EMP rồi chia hai vế cho nhau, mẫu số chung bị triệt tiêu:'),
        ('eq', 'P(OCC|r₁:t)/P(EMP|r₁:t) = [p(r_t|OCC)/p(r_t|EMP)] · [P(OCC|r₁:t−1)/P(EMP|r₁:t−1)]'),
        ('p', 'Lấy log: l_t = l_{t−1} + log-likelihood-ratio. Dùng mô hình cảm biến ngược p(OCC | r_t) với tiền nghiệm 0.5 (l₀ = 0) thì số hạng cộng chính là log-odds của mô hình cảm biến '
              'ngược, ta được (2.8). Ưu điểm: chỉ là phép cộng, ổn định số, không phải chuẩn hóa. Hess et al. (2016) viết cùng ý dưới dạng nhân odds: M_new = clamp(odds⁻¹(odds(M_old)·odds(p_hit))).'),
        ('src', 'Elfes (1989), mục “Occupancy grid” và “Bayesian estimation”, tr. 47–49; Hess et al. (2016), mục IV.B (hits/misses, odds), công thức (2); Corke et al. (2023), mục 6.3, tr. 249–252.'),
        ('code', 'buildMap(): lh = log(.7/.3); lm = log(.4/.6); cộng lm cho các ô trên tia (miss), lh cho ô điểm cuối (hit), kẹp trong [−4, 4]; mapMetrics(): ngưỡng P ≥ 0.65.')])
    h3('2.3.2. Mô hình nhiễu cảm biến và quyết định MAP')
    para(orig('Trong thực tế, cảm biến đo khoảng cách chịu ảnh hưởng'))
    eq('p(r | z, θ) = 1/(2π σ_r σ_θ) · exp{ −½ [ (r − z)²/σ_r² + θ²/σ_θ² ] }', 2)
    para(orig('Bên cạnh việc lập bản đồ từ cảm biến, Elfes (1989)'))

    h2('2.4. Front-end LiDAR: so khớp quét và phát hiện đóng vòng')
    para(orig('Cơ chế hoạt động của đối tượng lidarSLAM'))
    h3('2.4.1. Biểu diễn quét LiDAR, bản đồ con và cập nhật theo hàm odds')
    para(orig('Theo Hess et al. (2016), mỗi lần quét LiDAR'))
    eq('T_ξ(p) = R_ξ p + t_ξ', 2)
    para(orig('Một nhóm các lần quét liên tiếp được ghép lại'))
    eq('M_new(x) = clamp( odds⁻¹( odds(M_old(x)) · odds(p_hit) ) )', 2)
    h3('2.4.2. So khớp quét vào bản đồ con và phát hiện đóng vòng bằng Nhánh – Cận')
    para(orig('Trước khi chèn quét mới vào Submap hiện tại'))
    eq('argmin_ξ Σ_k ( 1 − M_smooth(T_ξ(h_k)) )² ;     ξ* = argmax_{ξ ∈ W} Σ_k M_nearest(T_ξ(h_k))', 2)
    para(orig('Do việc chỉ ghép quét vào Submap cục bộ'))
    para(orig('Nếu score(c) thấp hơn ngưỡng điểm đóng vòng'))
    h3('2.4.3. So khớp ICP điểm–đường và hiệp phương sai của phép so khớp (phương pháp được cài đặt)')
    para('Để tự cài đặt và có được ma trận thông tin Ω_ij cho từng cạnh, front-end của đề tài dùng ICP điểm–đường. Với quét nguồn {p_k} (nút j) và quét đích {q_k} có pháp tuyến n_k '
         '(nút i, ước lượng từ 2 điểm lân cận), tìm tư thế tương đối z = (t, φ):')
    eq('z* = argmin_z Σ_k ρ( n_kᵀ ( R(φ) p_k + t − q_k ) )', 2)
    para('với ρ là hàm Huber (ngưỡng 5 cm); bài toán giải lặp Gauss–Newton với Jacobian mỗi điểm J_k = [n_kᵀ , n_kᵀ ∂R(φ)/∂φ p_k]. Tại nghiệm, hiệp phương sai xấp xỉ bằng nghịch đảo '
         'Hessian (xấp xỉ Laplace, Censi, 2007):')
    EQ_ICPCOV = eq('Σ_z ≈ s² (Jᵀ W J)⁻¹ ,     s² = Σ_k r_k² / (n − 3) ,     Ω_ij = Σ_z⁻¹', 2)
    para('Cách này phản ánh đúng hình học: trong hành lang dài, ràng buộc dọc hành lang yếu nên Σ_z lớn theo phương đó (hiện tượng suy biến mà Kümmerle et al., 2009 mô tả). '
         f'Cạnh đóng vòng được chấp nhận khi tỷ lệ điểm khớp (< 15 cm) > {int(100*CFG["lc_min_inlier"])} % và RMS phần dư < {100*CFG["lc_max_rms"]:.0f} cm.')
    ext('Tuyến tính hóa ICP điểm–đường và ước lượng hiệp phương sai', [
        ('p', 'Phần dư điểm–đường r_k = n_kᵀ(R(φ)p_k + t − q_k) đo khoảng cách có dấu từ điểm đã biến đổi tới tiếp tuyến bề mặt tại q_k – cho phép điểm “trượt” dọc bề mặt, nên hội tụ nhanh '
              'và chính xác hơn điểm–điểm khi hai lần quét lấy mẫu bề mặt ở các vị trí khác nhau.'),
        ('eq', '∂r_k/∂t = n_kᵀ ,   ∂r_k/∂φ = n_kᵀ R′(φ) p_k ,   R′(φ) = [−sinφ  −cosφ ; cosφ  −sinφ]'),
        ('p', 'Gauss–Newton: (Jᵀ W J) Δz = −Jᵀ W r, z ← z + Δz; W là trọng số Huber (w = 1 nếu |r| ≤ 5 cm, ngược lại 5cm/|r|). Ngưỡng ghép cặp giảm dần (1.2 m → 0.25 m) để thoát khỏi cặp sai ban đầu.'),
        ('p', 'Hiệp phương sai: gần nghiệm, −log p(z) ≈ (1/2s²) Σ r_k² ≈ const + ½ Δzᵀ (JᵀWJ/s²) Δz ⇒ Σ_z ≈ s²(JᵀWJ)⁻¹. Censi (2007) chỉ ra xấp xỉ này bỏ qua sai số do việc ghép cặp '
              'và độ cong bề mặt, nên có thể lệch – đúng hiện tượng đề tài quan sát ở nhiễu lớn (mục 4.5).'),
        ('src', 'Censi (2007), mục II–III; Corke et al. (2023), mục 6.3.2 (scan matching, tr. 253–256) và Phụ lục F.2 (nonlinear least squares, tr. 739–742); Kümmerle et al. (2009), mục 5 (corridor).'),
        ('code', 'icpPL(): r = sum((Pp(m,:) − q).*nq, 2); Jc = [nq, sum(nq.*dP,2)]; Huber w; res.cov = s2*inv(Hh); scanNormals() ước lượng pháp tuyến.')])

    h2('2.5. Tối ưu hóa đồ thị tư thế (Back-end)')
    para(orig('Sau khi khối Front-end tạo ra các đỉnh tư thế'))
    h3('2.5.1. Hàm mục tiêu bình phương tối thiểu phi tuyến')
    para(orig('Theo Grisetti et al. (2010), gọi x = (x_1^T'))
    EQ_EIJ = eq('e_ij(x_i, x_j) = [ R_ijᵀ ( R_iᵀ (t_j − t_i) − t_ij ) ;  wrapToPi(θ_j − θ_i − θ_ij) ]', 2)
    para('Cách viết này trùng với dạng ma trận của Corke et al. (2023, công thức 6.33): f_k = [T_z⁻¹(z_ij) T⁻¹(ξ_i) T(ξ_j)]_xyθ – độ lệch giữa điều robot đo được (z_ij) và điều đồ thị đang suy ra (ξ_i ⊖ ξ_j).')
    para(orig('trong đó R_i và R_ij là các ma trận quay 2x2'))
    EQ_F = eq('F(x) = Σ_{<i,j> ∈ C} e_ijᵀ Ω_ij e_ij ,     x* = argmin_x F(x)', 2)
    para('Corke et al. (2023) diễn giải F(x) như thế năng của kết cấu đàn hồi: mỗi cạnh là một lò xo có độ cứng Ω_ij, tối ưu là “thả” đồ thị về năng lượng nhỏ nhất. '
         + orig('Trong môi trường nhà xưởng có tính đối xứng cục bộ cao'))
    ext('Suy dẫn sai số cạnh từ ma trận SE(2)', [
        ('p', 'Biểu diễn tư thế bằng ma trận đồng nhất T(ξ) = [R(θ) t ; 0 1]. Tư thế tương đối mà đồ thị “suy ra” là T_i⁻¹T_j; phép đo là T_z. Nếu hai giá trị khớp nhau thì T_z⁻¹T_i⁻¹T_j = I. '
              'Lấy thành phần (x, y, θ) của ma trận lỗi:'),
        ('eq', 'T_z⁻¹ T_i⁻¹ T_j = [ R_zᵀR_iᵀR_j   R_zᵀ(R_iᵀ(t_j − t_i) − t_z) ; 0  1 ]  ⇒  e_ij = [ R_zᵀ(R_iᵀ(t_j − t_i) − t_z) ; θ_j − θ_i − θ_z ]'),
        ('p', 'Đây là (2.13). Ω_ij = Σ_ij⁻¹ là trọng số: phép đo chính xác (Σ nhỏ) có “lò xo cứng”. Với cạnh odometry Ω = V⁻¹; với cạnh ICP Ω = Σ_z⁻¹ (2.12).'),
        ('src', 'Grisetti et al. (2010), mục IV, công thức (3)–(5) và Phụ lục (SE(2)); Corke et al. (2023), công thức (6.31)–(6.33), Hình 6.30, tr. 257–258.'),
        ('code', 'edgeJac(): “e = [cz.*v(:,1) + sz.*v(:,2), −sz.*v(:,1) + cz.*v(:,2), wrapPi(xj(:,3) − xi(:,3) − Z(:,3))]” (vector hóa cho mọi cạnh).')])
    h3('2.5.2. Tuyến tính hóa, cấu trúc thưa và phép lặp Gauss–Newton')
    para(orig('Để giải bài toán phi tuyến trên, các thuật toán Gauss-Newton'))
    eq('e_ij(x̃ + Δx) ≈ e_ij(x̃) + J_ij Δx ,     J_ij = ( 0 … A_ij … B_ij … 0 )', 2)
    EQ_AB = eq('A_ij = [ −R_ijᵀ R_iᵀ   R_ijᵀ (∂R_iᵀ/∂θ_i)(t_j − t_i) ; 0ᵀ  −1 ] ,   B_ij = [ R_ijᵀ R_iᵀ   0 ; 0ᵀ   1 ]', 2)
    para(orig('trong đó mỗi cạnh <i, j> chỉ đóng góp cộng dồn'))
    EQ_GN = eq('H = Σ J_ijᵀ Ω_ij J_ij ,   b = Σ J_ijᵀ Ω_ij e_ij ,   H Δx* = −b ,   x̃ ← x̃ ⊕ Δx*', 2)
    para(orig('Do sai số e_ij chỉ phụ thuộc vào vị trí tương đối'))
    ext('Suy dẫn A_ij, B_ij và phương trình chuẩn H Δx = −b', [
        ('p', 'Đạo hàm theo t_i: số hạng R_zᵀR_iᵀ(t_j − t_i) có ∂/∂t_i = −R_zᵀR_iᵀ; theo θ_i: chỉ R_iᵀ phụ thuộc θ_i nên ∂/∂θ_i = R_zᵀ(∂R_iᵀ/∂θ_i)(t_j − t_i), với ∂R_iᵀ/∂θ = [−sinθ cosθ ; −cosθ −sinθ]; '
              'thành phần góc θ_j − θ_i − θ_z có ∂/∂θ_i = −1. Tương tự theo nút j: ∂/∂t_j = R_zᵀR_iᵀ, ∂/∂θ_j = 1. Đây chính là (2.16); Corke (tr. 259) tính lại bằng Symbolic Math Toolbox.'),
        ('p', 'Thay xấp xỉ tuyến tính vào F: F(x̃ + Δx) ≈ c + 2bᵀΔx + ΔxᵀHΔx. Đặt gradient bằng 0: ∂F/∂Δx = 2b + 2HΔx = 0 ⇒ HΔx* = −b (Grisetti, công thức 13–15). '
              'Vì J_ij chỉ khác 0 ở cột của nút i và j, mỗi cạnh chỉ cộng vào 4 khối H_ii, H_ij, H_ji, H_jj và 2 khối b_i, b_j ⇒ H rất thưa (Hình 4.5b).'),
        ('p', 'Tự do chuẩn (gauge freedom): F không đổi khi cả đồ thị bị tịnh tiến/quay cùng nhau ⇒ H suy biến hạng 3. Cộng P₀⁻¹ vào H_[11] (neo nút 1) làm H khả nghịch, giống '
              '“FirstNodePose” của optimizePoseGraph trong sách (tr. 261).'),
        ('src', 'Grisetti et al. (2010), công thức (6)–(16) và (17)–(19) (cấu trúc thưa của J, H, b); Corke et al. (2023), tr. 258–261 và Phụ lục F.2.4 “Sparse nonlinear least squares”, tr. 741–742.'),
        ('code', 'optimizePoseGraph(): pagemtimes tính AᵀΩA…; blocks() + sparse() ráp H thưa; accumarray() ráp b; “H(1:3,1:3) += Ainfo”; “dx = −(H\\b)”; dừng khi max|Δx| < 1e−6.')])
    h3('2.5.3. Hiệp phương sai của nghiệm và quan hệ với dead reckoning')
    para('Ở nghiệm x*, F(x* + Δx) ≈ F(x*) + ΔxᵀHΔx, nên hậu nghiệm có dạng Gauss với ma trận thông tin H. Hiệp phương sai toàn quỹ đạo và hiệp phương sai biên của từng nút là:')
    EQ_SIG = eq('Σ = H⁻¹ ,     Σ_kk = [H⁻¹]_{3k−2:3k , 3k−2:3k} ,     σ_x,k = √Σ_kk(1,1)', 2)
    para('Đây là cầu nối giữa slide (EKF, P) và pose-graph: thay P của EKF bằng Σ_kk để vẽ hai đường biên ±2σ và elip 95 % cho SLAM. **Nếu đồ thị chỉ có các cạnh odometry nối tiếp '
         '(không đóng vòng) thì nghiệm trùng dead reckoning và Σ_kk trùng P_k của EKF**; đề tài kiểm chứng bằng số ở mục 4.2. Khi có cạnh đóng vòng, H nhận thêm khối ngoài chuỗi, '
         'các phần tử của H⁻¹ giảm: độ bất định được “ghìm” lại.')
    ext('Xấp xỉ Laplace và chứng minh “chuỗi odometry ≡ dead reckoning”', [
        ('p', 'Xấp xỉ Laplace: p(x|z) ∝ exp(−½F(x)). Khai triển bậc 2 quanh x*: ½F ≈ ½F(x*) + ½ΔxᵀHΔx (gradient = 0 tại cực tiểu; H = JᵀΩJ là xấp xỉ Gauss–Newton của Hessian của ½F). '
              'Vậy p(x|z) ≈ N(x*, H⁻¹). Điều này đúng tới bậc 2, nên càng chính xác khi phi tuyến yếu và nhiễu nhỏ.'),
        ('p', 'Chuỗi odometry: chỉ có N−1 cạnh cho N−1 nút mới ⇒ nghiệm x_k = x_{k−1} ⊕ z_k làm mọi e = 0, tức F = 0 = cực tiểu toàn cục ⇒ trùng dead reckoning. Về hiệp phương sai: tuyến tính hóa '
              'chuỗi cho Δx_k = F_x Δx_{k−1} + F_v w_k với w_k ~ N(0, V) độc lập – đây là một quá trình Gauss–Markov mà hiệp phương sai biên thỏa đúng truy hồi P_k = F_x P_{k−1} F_xᵀ + F_v V F_vᵀ. '
              'Phép nghịch đảo H (ma trận khối ba đường chéo) cho đúng các P_k này.'),
        ('p', f'Kiểm chứng số: max|x_pose-graph(chỉ odometry) − x_DR| = {info["dr_vs_odopg_maxdiff"]:.1e} m (sai số làm tròn máy).'),
        ('src', 'Grisetti et al. (2010), mục IV (ma trận thông tin H); Bishop, Pattern Recognition and Machine Learning, mục 4.4 (Laplace approximation) – tài liệu đọc thêm; '
                'Corke et al. (2023), tr. 261 (pose graph cho tư thế tương đối tối ưu, cần neo nút).'),
        ('code', 'margCov(): S = inv(full(H)); Pc(:,:,k) = S(3k−2:3k, 3k−2:3k); trong runSLAM(): R.drDiff = max|Xo_ − Xdr| với đồ thị chỉ cạnh odometry.')])

    h2('2.6. Cơ sở lý thuyết về đánh giá định lượng độ chính xác SLAM')
    para(orig('Theo Kümmerle et al. (2009), trong cộng đồng nghiên cứu robot'))
    h3('2.6.1. Sai số quỹ đạo tuyệt đối (ATE) và sai số tương đối (RPE)')
    para(orig('Gọi x_{1:T} là chuỗi tư thế robot được ước lượng'))
    eq('ε(x₁:T) = Σ_{t=1..T} (x_t ⊖ x*_t)²', 2)
    para(orig('trong đó (+) là toán tử hợp thành chuyển động chuẩn'))
    eq('e_k = √[ (x̂_k − x_k)² + (ŷ_k − y_k)² ] ,     RMSE_ATE = √[ (1/N) Σ e_k² ]', 2)
    para(orig('Tuy nhiên, Kümmerle et al. (2009) chỉ ra rằng'))
    eq('ε(δ) = (1/N) Σ_{i,j} [ trans(δ_ij ⊖ δ*_ij)² + rot(δ_ij ⊖ δ*_ij)² ]', 2)
    para(orig('Trong mô hình đồ thị tư thế, năng lượng biến dạng'))
    h3('2.6.2. Tính nhất quán bản đồ và chỉ số ô lưới (Map Accuracy, Obstacle IoU)')
    para(orig('Kümmerle et al. (2009) phân biệt hai cấp độ'))
    bullet(orig('Tính nhất quán cục bộ (Local Consistency)')); bullet(orig('Tính nhất quán toàn cục (Global Consistency)'))
    para(orig('Dựa trên quy tắc phân ngưỡng MAP của Elfes'))
    eq('Map Accuracy = (TP + TN)/(TP + TN + FP + FN) × 100 % ,     Obstacle IoU = TP/(TP + FP + FN) × 100 %', 2)
    h3('2.6.3. Kiểm định nhất quán thống kê: biên 95 %, NEES và ANEES')
    para('ATE cho biết sai số lớn hay nhỏ, nhưng không cho biết bộ ước lượng có “tự đánh giá đúng” độ bất định hay không. Theo slide (Ch.4 part 1, mục 95 % confidence) và Bar-Shalom et al. (2001):')
    bullet('Tỷ lệ % mẫu |x̃_k| ≤ 2σ_x,k (tương tự y, θ): lý thuyết ≈ 95.4 %.')
    bullet('NEES vị trí ε_k = ẽ_kᵀ P_xy,k⁻¹ ẽ_k ~ χ²₂ (kỳ vọng 2); tỷ lệ ε_k ≤ 5.991 là tỷ lệ điểm nằm trong elip 95 %.')
    bullet('ANEES Monte Carlo: M·ε̄_k ~ χ²_{2M}; khoảng chấp nhận 95 % là [χ²_{2M}(0.025)/M , χ²_{2M}(0.975)/M]. ANEES ≈ 2: nhất quán; lớn hơn biên trên: quá tự tin; nhỏ hơn biên dưới: quá thận trọng.')
    ext('Vì sao NEES có phân bố χ² và cách lập biên Monte Carlo', [
        ('p', 'Nếu bộ ước lượng nhất quán thì sai số ẽ_k ~ N(0, P_k). Như ở khung mở rộng về elip, ẽᵀP⁻¹ẽ ~ χ²_n với n = 2 (vị trí) ⇒ E[ε_k] = 2. Một lần chạy cho ε_k dao động mạnh, nên chạy M lần '
              'độc lập và lấy trung bình: tổng M biến χ²₂ độc lập là χ²_{2M} ⇒ M·ε̄_k ~ χ²_{2M}.'),
        ('p', f'Với M = {MC.get("M", 20)}: biên 95 % = [{MC["bounds"][0]:.2f}, {MC["bounds"][1]:.2f}] (tính bằng chi2inv của MATLAB). Đây là phép kiểm định giả thuyết H₀: “bộ ước lượng nhất quán”.'),
        ('p', 'Lưu ý: % mẫu trong biên từng trục và NEES dùng cùng P nhưng NEES xét đồng thời x, y (kể cả tương quan), nên nghiêm ngặt hơn.'),
        ('src', 'Bar-Shalom, Li & Kirubarajan (2001), mục 5.4 “Consistency of state estimators”; Slide Ch.4 part 1, tr. 96–100; Kümmerle et al. (2009), mục 3 (công thức 1–3) cho ATE/RPE.'),
        ('code', 'metrics(): nees(k) = e(k,1:2)/P(1:2,1:2,k)*e(k,1:2)′; chi2q() → biên ANEES; KB2 trong mục 5 của main_pose_graph_slam.m.')])

    h2('2.7. Phân tích, so sánh các phương pháp định vị – lập bản đồ')
    table('2.1', 'So sánh định tính các phương pháp (tổng hợp từ slide, Corke 2023, Grisetti 2010, Hess 2016)',
          ['Tiêu chí', 'Dead reckoning (EKF dự đoán)', 'EKF-SLAM (landmark)', 'Ghép quét LiDAR không đóng vòng', 'Pose-Graph SLAM'],
          [['Thông tin dùng', 'Encoder', 'Odometry + mốc', 'Odometry + LiDAR liên tiếp', 'Odometry + LiDAR + đóng vòng'],
           ['Bất định theo thời gian', 'Tăng không giới hạn', 'Giảm khi thấy lại mốc', 'Tăng chậm hơn DR', 'Bị chặn, co lại khi đóng vòng'],
           ['Độ phức tạp', 'O(1)/bước', 'O(M²) bộ nhớ', 'O(K) mỗi quét', 'Hệ thưa, O(N)–O(N^1.5)'],
           ['Sửa quá khứ', 'Không', 'Gián tiếp', 'Không', 'Có (làm trơn toàn quỹ đạo)'],
           ['Rủi ro chính', 'Trôi', 'Gán dữ liệu sai', 'Trôi dài hạn, suy biến hành lang', 'Đóng vòng sai'],
           ['Vai trò trong đề tài', 'Đối chứng (A)', 'Lý thuyết nền', 'Đối chứng (B)', 'Giải pháp (C)']],
          widths=[2.8, 3.3, 3.0, 3.3, 3.6], size=10)

    # ======================================================== CHƯƠNG 3
    EQN[0] = 0
    h1('CHƯƠNG 3. NỘI DUNG NGHIÊN CỨU VÀ THIẾT KẾ MÔ PHỎNG')
    h2('3.1. Phát biểu bài toán và yêu cầu kỹ thuật')
    para(orig('Tình huống ứng dụng cụ thể'))
    para('**Đầu vào:**', indent=False)
    bullet('Bản đồ chuẩn M_true (lưới 400 × 400 ô, ô 5 cm) – chỉ dùng để mô phỏng cảm biến và chấm điểm; thuật toán SLAM không được biết.')
    bullet(f'Quỹ đạo thật từ Simulink sl_drivepoint ({info["n_dense"]} mẫu, {info["t_sim"]:.1f} s, {info["path_len"]:.1f} m), rút gọn còn N = {info["N"]} keyframe (nút đồ thị).')
    bullet('Odometry có nhiễu giữa các keyframe; quét LiDAR 2D 240 tia/vòng có nhiễu khoảng cách σ_r và nhiễu góc σ_β.')
    para('**Đầu ra:**', indent=False)
    bullet('Quỹ đạo ước lượng X̂ và hiệp phương sai biên Σ_kk từng tư thế (để vẽ biên ±2σ, elip 95 %); đồ thị tư thế tối ưu; bản đồ lưới M_est.')
    bullet('Các chỉ số: ATE RMSE, RPE, % mẫu trong biên 95 %, ANEES, Map Accuracy, Obstacle IoU, số cạnh đóng vòng; dữ liệu CSV cho demo 3D.')
    para('**Giả thiết:** thế giới tĩnh; nhiễu Gauss trắng không có sai số hệ thống; tư thế ban đầu biết trước (neo nút 1).')
    para('**Yêu cầu kỹ thuật:** ATE RMSE (SLAM) < 0.15 m ở σ_r = 3 cm; Map Accuracy > 90 %; ≥ 90 % mẫu sai số trong biên ±2σ.')
    h2('3.2. Phương pháp tiếp cận và phân định đóng góp')
    para('**Lý do lựa chọn:** (i) pose-graph là hướng chuẩn cho SLAM LiDAR trong sách (Corke et al., 2023, mục 6.4.2) và vượt EKF-SLAM về khả năng mở rộng; (ii) tự cài đặt back-end '
         'để truy cập ma trận thông tin H, tính Σ = H⁻¹ và đánh giá độ tin cậy 95 % – điều lidarSLAM không cung cấp; (iii) dùng mô hình Simulink có sẵn của sách để quỹ đạo tuân theo '
         'động học xe thực tế (không thể quay tại chỗ, có bán kính quay tối thiểu); (iv) Ground Truth cho phép đánh giá tuyệt đối. Quy trình ở Hình 3.1.')
    figure('fig_luu_do_he_thong.png', '3.1', 'Lưu đồ quy trình: Ground Truth, Simulink, cảm biến, SLAM, đánh giá và demo 3D', 16)
    table('3.1', 'Phân định nội dung kế thừa và nội dung nhóm tự thực hiện', ['Hạng mục', 'Kế thừa từ tài liệu / công cụ', 'Nhóm tự thiết kế, cài đặt, đánh giá'],
          [['Lý thuyết', 'Odometry, EKF, ±2σ, elip (slide Ch.4, Corke 2023); occupancy grid (Elfes); submap & BnB (Hess); pose-graph (Grisetti); thước đo (Kümmerle); NEES (Bar-Shalom)',
            'Kết nối thành khung thống nhất; chỉ ra và kiểm chứng số quan hệ Σ = H⁻¹ ↔ P của EKF'],
           ['Chuyển động robot', 'Mô hình Simulink sl_drivepoint + Bicycle Kinematic Model (RVC3, Corke 2023)', 'Chạy nối tiếp qua 11 waypoint nhà xưởng, trích keyframe, kiểm tra va chạm'],
           ['Môi trường, cảm biến', 'Ý tưởng binaryOccupancyMap, rangeSensor', 'Tự thiết kế mặt bằng 20×20 m; tự viết bộ dò tia LiDAR có nhiễu'],
           ['Front-end / Back-end', 'Nguyên lý ICP (Censi 2007); công thức e_ij, A_ij, B_ij (Grisetti 2010)', 'Tự cài đặt ICP điểm–đường có Huber, tiêu chí đóng vòng; Gauss–Newton với H thưa, neo nút, Σ = H⁻¹'],
           ['Đánh giá, trực quan', 'plotvehicle (RVC3); lidarSLAM để đối chiếu', 'Biểu đồ sai số + 2 đường biên 95 %, elip 95 %, Monte Carlo NEES/ANEES, Map Accuracy/IoU, độ nhạy σ_r; demo 3D PyVista']],
          widths=[2.6, 6.2, 7.2], size=10)
    h2('3.3. Thiết kế chi tiết')
    h3('3.3.1. Môi trường nhà xưởng')
    para('Mặt bằng gồm 4 tường bao dày 0.25 m, 4 khu kệ/máy CNC (A, B, C, D) có nhánh phụ phá đối xứng, 4 trụ kỹ thuật. Các hành lang tại X = 2.5; 10; 17.5 m và Y = 2.5; 10; 17.5 m. '
         'Quỹ đạo đi qua 11 waypoint, gồm đoạn đi–về hành lang phía Tây (đóng vòng ngắn) và quay về cửa kho (đóng vòng dài).')
    h3('3.3.2. Sinh quỹ đạo thật bằng mô hình Simulink sl_drivepoint')
    figure('fig0_simulink_sl_drivepoint.png', '3.2', 'Mô hình Simulink sl_drivepoint: bộ điều khiển đi tới điểm và khối Bicycle Kinematic Model', 15.5, 'Corke et al., 2023, Hình 4.7 – RVC3 Toolbox')
    para('Mô hình gồm khối Bicycle Kinematic Model (động học xe đạp) và bộ điều khiển tỉ lệ “đi tới điểm” (Corke et al., 2023, mục 4.1.1):')
    EQ_BIC = eq('ẋ = v cos θ ,   ẏ = v sin θ ,   θ̇ = (v/L) tan γ', 3)
    EQ_CTL = eq('v* = K_v √[(x* − x)² + (y* − y)²] ,   θ* = atan2(y* − y, x* − x) ,   γ = K_h (θ* ⊖ θ)', 3)
    para('với K_v = 0.5, K_h = 2, chiều dài cơ sở L = 1 m, vận tốc giới hạn [0, 1] m/s, góc lái tối đa 1 rad (bán kính quay tối thiểu L/tan1 ≈ 0.64 m); mô phỏng dừng khi cách đích < 0.1 m. '
         'Đề tài chạy mô hình lần lượt cho 10 đoạn (đích xg là waypoint kế tiếp, trạng thái đầu x0 là trạng thái cuối đoạn trước), bước tích phân tối đa 0.02 s. Từ quỹ đạo dày, '
         f'một keyframe mới được tạo khi robot đi được ≥ {CFG["kf_dist"]} m hoặc quay ≥ {CFG["kf_angle_deg"]:.0f}° (Corke, mục 6.4.2: “thêm nút mỗi khoảng một mét hoặc sau khúc cua gắt”). '
         f'Kết quả: {info["N"]} keyframe, số mẫu nằm trong vật cản = {info["collisions"]} (không va chạm).')
    ext('Mô hình xe đạp và luật điều khiển đi tới điểm', [
        ('p', 'Giả thiết bánh xe lăn không trượt ngang (B v = (v, 0)), điểm tham chiếu chuyển động tròn quanh tâm quay tức thời với bán kính R_B = L/tanγ, nên θ̇ = v/R_B = (v/L)tanγ – công thức (3.1). '
              'Ràng buộc không trượt ngang ẏ cosθ − ẋ sinθ = 0 là lý do robot **không thể** quay tại chỗ hay đi ngang (nonholonomic).'),
        ('p', 'Bộ điều khiển (3.2) là hai vòng tỉ lệ độc lập: vận tốc tỉ lệ khoảng cách tới đích (chậm dần khi tới gần), góc lái tỉ lệ sai lệch hướng. Phép ⊖ cho góc lấy trong (−π, π] (khối angdiff). '
              'Đây là lý do quỹ đạo ở góc cua là đường cong và robot giảm tốc tại mỗi waypoint.'),
        ('src', 'Corke et al. (2023), mục 4.1.1 Car-Like Vehicle: công thức (4.1)–(4.5), tr. 130–132; (4.6)–(4.7) và “Moving to a Point”, Hình 4.7–4.8, tr. 133–135; mô hình sl_drivepoint.slx trong RVC3-MATLAB-main/toolbox.'),
        ('code', 'mục 2 của main_pose_graph_slam.m: vòng lặp “x0 = Xd(end,:); xg = WP(i,:); out = sim(''sl_drivepoint'', ''StopTime'',''80'', ''MaxStep'',''0.02'')” và đoạn trích keyframe.')])
    h3('3.3.3. Front-end (thuật toán 1)')
    for t in ['Thêm nút k với dự đoán x̂_k = x̂_{k−1} ⊕ ẑ_k và cạnh odometry Ω_odo = V⁻¹.',
              f'ICP điểm–đường quét k với quét k−1 (khởi tạo bằng odometry); nếu tỷ lệ khớp > 70 % và RMS < {100*CFG["lc_max_rms"]:.0f} cm thì thêm cạnh ghép quét Ω = Σ_z⁻¹ theo {EQ_ICPCOV}.',
              f'Lan truyền hiệp phương sai nút mới: Σ_k = J₁ Σ_{{k−1}} J₁ᵀ + J₂ Σ_z J₂ᵀ (cùng dạng {EQ_EKF}).',
              f'Mỗi {2} nút: chọn tối đa 2 nút cũ j (k − j ≥ {CFG["lc_min_gap"]}) gần nhất trong bán kính {CFG["lc_radius"]} m; ICP với 3 góc khởi tạo (0, ±5°); chấp nhận nếu khớp > {int(100*CFG["lc_min_inlier"])} % và RMS < {100*CFG["lc_max_rms"]:.0f} cm.',
              'Có cạnh đóng vòng mới → back-end tối ưu toàn đồ thị, cập nhật X̂ và Σ.']:
        bullet(t)
    h3('3.3.4. Back-end (thuật toán 2 – Gauss–Newton)')
    for t in [f'Tính e_ij theo {EQ_EIJ}, A_ij, B_ij theo {EQ_AB} cho mọi cạnh (vector hóa bằng pagemtimes).',
              'Ráp H (ma trận thưa, sparse) và b; neo nút 1: H_[11] += P₀⁻¹ với P₀ = diag(10⁻⁶, 10⁻⁶, 10⁻⁸).',
              'Giải H Δx = −b, cập nhật x ← x ⊕ Δx; lặp đến khi max|Δx| < 10⁻⁶ (tối đa 30 vòng).',
              f'Tính Σ = H⁻¹ theo {EQ_SIG} → σ_x, σ_y, σ_θ và elip 95 % của từng nút.']:
        bullet(t)
    para('Phương pháp (B) dùng đúng các cạnh của (C) nhưng bỏ cạnh đóng vòng; (A) chỉ dùng odometry (và được kiểm chứng trùng EKF). Bản đồ của mỗi phương pháp được dựng bằng log-odds '
         'hits/misses; ô “đã quan sát” nếu |P − 0.5| > 0.05, là vật cản nếu P ≥ 0.65; bản đồ dựng bằng tư thế thật làm cận trên.')
    h2('3.4. Công cụ, thông số và kịch bản thử nghiệm')
    table('3.2', 'Thông số thiết lập hệ thống mô phỏng', ['Nhóm', 'Tham số', 'Giá trị', 'Ý nghĩa'],
          [['Môi trường', 'Kích thước / độ phân giải', '20 × 20 m, 5 cm (400 × 400 ô)', 'Mặt bằng phân xưởng'],
           ['Simulink', 'K_v, K_h, L, v_max, γ_max', '0.5, 2, 1 m, 1 m/s, 1 rad', 'sl_drivepoint (Corke 2023)'],
           ['Keyframe', 'Ngưỡng tịnh tiến / quay', f'{CFG["kf_dist"]} m / {CFG["kf_angle_deg"]:.0f}°', f'{info["N"]} nút, {info["path_len"]:.1f} m'],
           ['Odometry', 'σ_d , σ_ngang , σ_θ', f'{CFG["sigma_d"]} m, {CFG["sigma_lat"]} m, {CFG["sigma_th_deg"]:.1f}°', 'V mỗi keyframe'],
           ['LiDAR', 'Tầm đo / độ phân giải góc', f'{CFG["max_range"]:.0f} m / {CFG["ang_res_deg"]:.1f}° (240 tia)', 'Quét 360°'],
           ['', 'σ_r , σ_β', f'{CFG["sigma_r"]} m (khảo sát 0.01–0.08), {CFG["sigma_b_deg"]:.1f}°', 'Nhiễu khoảng cách, góc'],
           ['Front-end', 'Bán kính / khoảng cách nút LC', f'{CFG["lc_radius"]} m / ≥ {CFG["lc_min_gap"]} nút', 'Vùng tìm đóng vòng'],
           ['', 'Ngưỡng chấp nhận', f'khớp > {int(100*CFG["lc_min_inlier"])} %, RMS < {CFG["lc_max_rms"]} m', 'Loại đóng vòng sai'],
           ['Back-end', 'Thuật toán / dừng', 'Gauss–Newton, max|Δx| < 1e-6', 'H thưa, neo nút 1'],
           ['Bản đồ', 'l_hit, l_miss, ngưỡng', 'log(0.7/0.3), log(0.4/0.6), P ≥ 0.65', 'Quyết định MAP'],
           ['Đánh giá', 'Biên tin cậy', '±2σ (95.4 %), elip χ²₂ = 5.991', 'Theo slide Ch.4']], widths=[2.4, 4.2, 5.0, 4.4], size=10)
    para('**Các kịch bản:**', indent=False)
    bullet(f'KB1 – So sánh (A), (B), (C) tại mức nhiễu chuẩn, 1 lần chạy đại diện (seed = {CFG["seed"]}).')
    bullet(f'KB2 – Monte Carlo M = {MC["M"]} lần chạy độc lập (khác nhiễu) để kiểm định nhất quán 95 %.')
    bullet(f'KB3 – Độ nhạy theo σ_r ∈ {{1, 3, 5, 8}} cm, mỗi mức {CFG["M_sens"]} lần chạy.')
    bullet('KB4 – Đối chiếu với lidarSLAM (Navigation Toolbox) trên cùng dữ liệu quét.')
    para('**Công cụ:** MATLAB R2024b, Simulink, RVC3 Toolbox, Navigation Toolbox – chương trình main_pose_graph_slam.m (một tệp, gồm các hàm cục bộ). Python 3 + PyVista – demo_3d.py đọc CSV do MATLAB xuất.')

    # ======================================================== CHƯƠNG 4
    EQN[0] = 0
    h1('CHƯƠNG 4. KẾT QUẢ VÀ THẢO LUẬN')
    h2('4.1. Kết quả mô hình Simulink, môi trường và cảm biến')
    para(f'Hình 1.1a cho thấy quỹ đạo do sl_drivepoint sinh ra đi qua đủ 11 waypoint, cua tròn tại các góc (bán kính quay giới hạn) và không va chạm vật cản; {info["N"]} keyframe phân bố dày hơn '
         'ở khúc cua do ngưỡng góc. Hình 1.1b minh họa một lần quét: tia bị chặn tại tường và kệ, điểm phản xạ phân tán nhẹ do nhiễu Gauss; tia không gặp vật cản trong 12 m bị loại.')
    h2('4.2. Dead reckoning và hai đường biên 95 % (tái hiện kết quả trên slide)')
    figure('fig2_dead_reckoning_95.png', '4.1', 'Dead reckoning: (a) quỹ đạo ước lượng và elip tin cậy 95 %; (b) sai số tọa độ x với hai đường biên ±2σ', 16)
    para(f'Hình 4.1 tái hiện đúng hiện tượng trên slide 4.5.2.2: elip 95 % phình to dần, biên ±2σ mở rộng liên tục và không co lại. Sai số vị trí lớn nhất của dead reckoning là {dr["ate_max"]:.2f} m, '
         f'ATE RMSE = {dr["ate"]:.3f} m, độ bất định σ_pos đạt {dr["sig_max"]:.2f} m. Ở lần chạy đại diện này, {dr["in_x"]:.0f} % / {dr["in_y"]:.0f} % / {dr["in_t"]:.0f} % số mẫu x / y / θ nằm trong biên ±2σ. '
         + ('Tỷ lệ của trục x thấp hơn nhiều so với 95 % không có nghĩa EKF sai: sai số dead reckoning **tương quan rất mạnh theo thời gian** (một lệch góc sớm kéo lệch toàn bộ quãng đường phía sau), '
            'nên trong một lần chạy đơn lẻ đường sai số có thể nằm ngoài biên suốt một đoạn dài. Tính nhất quán phải được đánh giá thống kê qua nhiều lần chạy độc lập: Monte Carlo ở mục 4.4 cho '
            f'{MC["dr"]["in_x"]:.1f} / {MC["dr"]["in_y"]:.1f} / {MC["dr"]["in_t"]:.1f} % và ANEES = {MC["dr"]["anees"]:.2f} ≈ 2 – bộ ước lượng EKF không chính xác nhưng **nhất quán**, đúng kết luận của slide. '
            if min(dr['in_x'], dr['in_y'], dr['in_t']) < 90 else
            'Dù sai số lớn, bộ ước lượng EKF không chính xác nhưng **nhất quán** – nó “biết” mình không chắc chắn, đúng kết luận của slide. ')
         + f'Đồ thị chỉ gồm cạnh odometry cho nghiệm trùng dead reckoning với sai khác lớn nhất {info["dr_vs_odopg_maxdiff"]:.1e} m, kiểm chứng nhận định ở mục 2.5.3.')
    h2('4.3. So sánh ba phương pháp (Kịch bản 1)')
    figure('fig3_quy_dao_elip95.png', '4.2', 'Quỹ đạo và elip tin cậy 95 %: (a) không đóng vòng; (b) pose-graph SLAM với các cạnh đóng vòng (màu hồng)', 16)
    para(f'Hình 4.2a: chỉ ghép quét liên tiếp đã tốt hơn DR rất nhiều (ATE {nl["ate"]:.3f} m so với {dr["ate"]:.3f} m) nhờ ràng buộc hình học từ LiDAR, nhưng elip vẫn phình dần (σ_pos cuối = {nl["sig_final"]:.3f} m). '
         f'Hình 4.2b: pose-graph SLAM tìm được {info["n_lc"]} cạnh đóng vòng (trên {info["n_lc_try"]} lần thử), ở đoạn đi–về hành lang phía Tây và khi quay về cửa kho; elip co rõ rệt, ATE còn {sl["ate"]:.3f} m.')
    figure('fig4_sai_so_bien95_3pp.png', '4.3', 'Sai số x, y, θ theo bước k và hai đường biên tin cậy 95 % (±2σ) của ba phương pháp (thang trục tung khác nhau)', 16.5)
    para('Hình 4.3 là kết quả trọng tâm. Mỗi ô vẽ sai số (đường liền) và **hai đường biên ±2σ** (nét đứt) lấy từ P của EKF (cột 1) hoặc từ Σ = H⁻¹ (cột 2, 3). Nhận xét:')
    bullet(f'Độ rộng biên giảm theo thứ tự DR → không đóng vòng → SLAM: ở cuối hành trình ±2σ_θ của DR là ±{2*dr["sig_th_final_deg"]:.1f}°, không đóng vòng ±{2*nl["sig_th_final_deg"]:.1f}°, SLAM ±{2*sl["sig_th_final_deg"]:.1f}°; '
           'biên vị trí của SLAM bị “bóp” lại tại các bước có đóng vòng.')
    bullet(f'Tỷ lệ mẫu trong biên: DR {dr["in_x"]:.0f}/{dr["in_y"]:.0f}/{dr["in_t"]:.0f} %, không đóng vòng {nl["in_x"]:.0f}/{nl["in_y"]:.0f}/{nl["in_t"]:.0f} %, SLAM {sl["in_x"]:.0f}/{sl["in_y"]:.0f}/{sl["in_t"]:.0f} % (x/y/θ).')
    bullet('Sai số của SLAM không tăng dần theo k mà dao động quanh 0 và bị chặn: sai số tích lũy đã được phân bổ lại trên toàn đồ thị (mô hình lò xo của Kümmerle et al., 2009).')
    ext('Cách đọc Hình 4.3 khi bảo vệ', [
        ('b', 'Đường liền là sai số thật (cần ground truth mới tính được); hai đường nét đứt là ±2σ do chính thuật toán tự ước lượng (không cần ground truth). So sánh hai thứ này = kiểm tra thuật toán “tự biết mình sai bao nhiêu”.'),
        ('b', 'Cột DR: biên nở hình “kèn” đúng như slide 4.5.2.2 (tr. 19). Cột SLAM: biên hẹp và có chỗ “thắt” tại bước đóng vòng – tương đương hiệu ứng quan sát landmark ở slide tr. 32–33.'),
        ('b', 'Nếu sai số vượt biên nhiều lần → bộ ước lượng quá tự tin (slide part 1, tr. 99) – xem kết quả ở nhiễu LiDAR lớn, mục 4.5.'),
        ('src', 'Slide Ch.4 part 2, tr. 19, 32–33; Slide Ch.4 part 1, tr. 96–100; Corke et al. (2023), Hình 6.4–6.5, tr. 224–225.')])
    figure('fig5_ate_va_do_bat_dinh.png', '4.4', '(a) Sai số vị trí tuyệt đối e_k theo bước k; (b) độ bất định vị trí √(σ_x² + σ_y²) (thang log)', 16)
    para(f'Hình 4.4a: e_k của DR tăng tới {dr["ate_max"]:.2f} m, không đóng vòng tới {nl["ate_max"]:.3f} m, SLAM chỉ tới {sl["ate_max"]:.3f} m; sai số điểm cuối lần lượt {dr["final"]:.3f} / {nl["final"]:.3f} / {sl["final"]:.3f} m. '
         'Hình 4.4b là “chữ ký” của SLAM: độ bất định tăng giữa các lần đóng vòng và sụt xuống khi đóng vòng xảy ra.')
    figure('fig6_do_thi_tu_the.png', '4.5', '(a) Đồ thị tư thế sau tối ưu; (b) cấu trúc thưa của H; (c) hội tụ F(x) qua các vòng lặp Gauss–Newton', 16.5)
    para(f'Đồ thị có {info["N"]} nút, {info["n_edges"]} cạnh ({info["n_odo"]} odometry, {info["n_seq"]} ghép quét, {info["n_lc"]} đóng vòng). H kích thước {3*info["N"]}×{3*info["N"]} chỉ có {info["H_density"]:.1f} % phần tử khác 0 '
         f'(dải chéo từ cạnh tuần tự, các khối rời từ đóng vòng). Khởi tạo bằng dead reckoning, F(x) giảm từ {info["chi2_init"]:.0f} xuống {info["chi2_final"]:.1f} sau {info["gn_iters"]} vòng lặp; '
         f'giá trị cuối so với số bậc tự do ≈ {info["dof"]} (tỷ số {info["chi2_final"]/info["dof"]:.2f}) cho thấy không có cạnh đóng vòng sai làm “căng” đồ thị.')
    figure('fig7_ban_do_luoi.png', '4.6', 'Bản đồ lưới: (a) Ground truth; (b) từ dead reckoning; (c) từ pose-graph SLAM; (d) đối chiếu từng ô (lục TP, đỏ FP, lam FN)', 16.5)
    para(f'Bản đồ từ DR bị nhân đôi tường, xoay lệch kệ – hiện tượng “bóng ma” nêu ở Chương 1; Obstacle IoU chỉ {dr["iou"]:.1f} %. Bản đồ từ SLAM sắc nét, Map Accuracy {sl["acc"]:.2f} %, IoU {sl["iou"]:.1f} %, '
         f'sát cận trên dựng bằng tư thế thật (Acc {S1["gtpose"]["acc"]:.2f} %, IoU {S1["gtpose"]["iou"]:.1f} %). FP nằm ở rìa bề mặt do nhiễu khoảng cách; FN là ô bề mặt bị bắn sượt; lõi khối đặc không quan sát được nên bị loại (xám).')
    rows = [['ATE RMSE [m]', f3(dr['ate']), f3(nl['ate']), f3(sl['ate'])], ['Sai số vị trí lớn nhất [m]', f3(dr['ate_max']), f3(nl['ate_max']), f3(sl['ate_max'])],
            ['Sai số tại điểm kết thúc [m]', f3(dr['final']), f3(nl['final']), f3(sl['final'])], ['RMSE góc hướng [°]', f2(dr['head_rmse']), f2(nl['head_rmse']), f2(sl['head_rmse'])],
            ['RPE Δ = 20 nút, tịnh tiến [m]', f3(dr['rpe20_t']), f3(nl['rpe20_t']), f3(sl['rpe20_t'])], ['σ_pos lớn nhất [m]', f3(dr['sig_max']), f3(nl['sig_max']), f3(sl['sig_max'])],
            ['% mẫu trong ±2σ (x / y / θ)'] + [f'{m["in_x"]:.0f} / {m["in_y"]:.0f} / {m["in_t"]:.0f}' for m in (dr, nl, sl)],
            ['% điểm trong elip 95 %', f1(dr['in_ell']), f1(nl['in_ell']), f1(sl['in_ell'])], ['ANEES (kỳ vọng 2)', f2(dr['anees']), f2(nl['anees']), f2(sl['anees'])],
            ['Map Accuracy [%]', f2(dr['acc']), f2(nl['acc']), f2(sl['acc'])], ['Obstacle IoU [%]', f1(dr['iou']), f1(nl['iou']), f1(sl['iou'])]]
    table('4.1', 'Kết quả định lượng ba phương pháp (σ_r = 3 cm, 1 lần chạy đại diện)', ['Chỉ số', '(A) Dead reckoning', '(B) Không đóng vòng', '(C) Pose-Graph SLAM'], rows, widths=[6.0, 3.3, 3.3, 3.4])
    para(f'So với dead reckoning, pose-graph SLAM giảm ATE {pct(dr["ate"], sl["ate"]):.1f} %; so với không đóng vòng giảm {pct(nl["ate"], sl["ate"]):.1f} %: LiDAR liên tiếp khử phần lớn trôi của encoder, '
         'đóng vòng khử phần trôi dài hạn còn lại và chặn độ bất định.')
    h2('4.4. Kiểm định độ nhất quán bằng Monte Carlo (Kịch bản 2)')
    figure('fig8_monte_carlo_nhat_quan.png', '4.7', f'Monte Carlo {MC["M"]} lần: (a) NEES vị trí trung bình và biên chấp nhận 95 %; (b) tỷ lệ sai số nằm trong biên 95 %', 16)
    mrow = [[nm, f'{MC[k]["ate_mean"]:.3f} ± {MC[k]["ate_std"]:.3f}', f'{MC[k]["in_x"]:.1f} / {MC[k]["in_y"]:.1f} / {MC[k]["in_t"]:.1f}', f1(MC[k]['in_ell']), f2(MC[k]['anees'])]
            for k, nm in [('dr', '(A) Dead reckoning'), ('nolc', '(B) Không đóng vòng'), ('slam', '(C) Pose-Graph SLAM')]]
    table('4.2', f'Kết quả Monte Carlo (M = {MC["M"]})', ['Phương pháp', 'ATE TB ± ĐLC [m]', '% trong ±2σ (x / y / θ)', '% trong elip 95 %', 'ANEES'], mrow, widths=[4.2, 3.4, 3.6, 2.6, 2.2])
    lo, hi = MC['bounds']
    def judge(a):
        return 'nằm trong khoảng (nhất quán)' if lo <= a <= hi else ('dưới biên dưới (thận trọng)' if a < lo else 'trên biên trên (quá tự tin)')
    para(f'Khoảng chấp nhận 95 % của NEES trung bình với M = {MC["M"]} là [{lo:.2f}, {hi:.2f}]. ANEES của DR = {MC["dr"]["anees"]:.2f}, {judge(MC["dr"]["anees"])}; '
         f'không đóng vòng = {MC["nolc"]["anees"]:.2f}, {judge(MC["nolc"]["anees"])}; SLAM = {MC["slam"]["anees"]:.2f}, {judge(MC["slam"]["anees"])}. '
         f'Tỷ lệ sai số của SLAM nằm trong biên ±2σ là {MC["slam"]["in_x"]:.1f} / {MC["slam"]["in_y"]:.1f} / {MC["slam"]["in_t"]:.1f} % (x / y / θ). Nếu ANEES dưới 2, nguyên nhân là hiệp phương sai ICP {EQ_ICPCOV} '
         'gộp cả sai số lượng tử hóa do hai lần quét lấy mẫu bề mặt ở các điểm khác nhau nên hơi thận trọng – xu hướng an toàn cho điều hướng. '
         f'Về độ chính xác, ATE trung bình: SLAM {MC["slam"]["ate_mean"]:.3f} m, không đóng vòng {MC["nolc"]["ate_mean"]:.3f} m, DR {MC["dr"]["ate_mean"]:.3f} m.')
    figure('fig8b_boxplot_ate.png', '4.8', f'Phân bố ATE RMSE qua {MC["M"]} lần Monte Carlo (thang log)', 10)
    h2('4.5. Khảo sát độ nhạy theo nhiễu LiDAR (Kịch bản 3)')
    figure('fig9_do_nhay_nhieu_lidar.png', '4.9', 'Ảnh hưởng của σ_r đến (a) ATE RMSE, (b) Map Accuracy / IoU, (c) số đóng vòng và tỷ lệ trong elip 95 %', 16.5)
    srow = [[f'{r["sigma_r"]*100:.0f} cm', f'{r["n_lc"]:.0f}', f'{r["ate_nolc"]:.3f} ± {r["ate_nolc_sd"]:.3f}', f'{r["ate_slam"]:.3f} ± {r["ate_slam_sd"]:.3f}',
             f2(r['acc_slam']), f1(r['iou_slam']), f1(r['in_ell'])] for r in SE]
    table('4.3', f'Độ nhạy theo σ_r (trung bình {CFG["M_sens"]} lần chạy)', ['σ_r', 'Số LC', 'ATE không đóng vòng [m]', 'ATE SLAM [m]', 'Map Acc. SLAM [%]', 'IoU SLAM [%]', '% trong elip 95 %'],
          srow, widths=[1.6, 1.5, 3.4, 3.0, 2.4, 2.0, 2.1], size=10)
    s_lo, s_hi = SE[0], SE[-1]
    worst = min(SE, key=lambda r: r['in_ell'])
    para(f'Khi σ_r tăng từ 1 lên 8 cm, ATE của SLAM tăng từ {s_lo["ate_slam"]:.3f} lên {s_hi["ate_slam"]:.3f} m, số đóng vòng từ {s_lo["n_lc"]:.0f} còn {s_hi["n_lc"]:.0f} (ngưỡng RMS loại bớt phép khớp kém) và IoU giảm từ '
         f'{s_lo["iou_slam"]:.1f} % xuống {s_hi["iou_slam"]:.1f} % do bề mặt tường “nhòe” dày hơn (probabilistic blurring, Elfes 1989). Ở mọi mức nhiễu SLAM luôn tốt hơn không đóng vòng '
         f'(ở 8 cm: {s_hi["ate_slam"]:.3f} m so với {s_hi["ate_nolc"]:.3f} m) và {"vẫn thỏa" if s_hi["ate_slam"] < 0.15 else "chưa thỏa"} yêu cầu ATE < 0.15 m.')
    para(f'Về độ tin cậy, tỷ lệ sai số nằm trong elip 95 % thấp nhất là {worst["in_ell"]:.1f} % tại σ_r = {100*worst["sigma_r"]:.0f} cm' +
         (' – thấp hơn mức danh định 95 %: ở nhiễu lớn hệ thống bắt đầu **quá tự tin**, vì xấp xỉ Σ_z ≈ s²(JᵀWJ)⁻¹ giả thiết phần dư độc lập và không phản ánh sai số hệ thống của việc ghép cặp, pháp tuyến bị nhiễu (Censi, 2007). '
          'Giải pháp: thổi phồng hiệp phương sai ICP theo mức nhiễu (tham số P.covInflate) hoặc siết ngưỡng chấp nhận đóng vòng.' if worst['in_ell'] < 95 else
          ' – vẫn đạt mức danh định 95 %, hệ thống nhất quán trên toàn dải nhiễu khảo sát.'))
    if TB:
        h2('4.6. Đối chiếu với lidarSLAM của Navigation Toolbox (Kịch bản 4)')
        figure('fig10_doi_chieu_lidarSLAM.png', '4.10', 'Quỹ đạo của lidarSLAM (tắt/bật đóng vòng) so với pose-graph tự cài đặt', 11)
        table('4.4', 'Đối chiếu với lidarSLAM trên cùng dữ liệu quét', ['Phương pháp', 'Số đóng vòng', 'ATE RMSE [m]', 'Có hiệp phương sai (biên 95 %)?'],
              [['lidarSLAM – tắt đóng vòng', f'{TB["off"]["n_lc"]}', f3(TB['off']['ate']), 'Không'],
               ['lidarSLAM – bật đóng vòng', f'{TB["on"]["n_lc"]}', f3(TB['on']['ate']), 'Không'],
               ['Pose-graph tự cài đặt (C)', f'{info["n_lc"]}', f3(sl['ate']), 'Có (Σ = H⁻¹)']], widths=[5.2, 2.6, 2.8, 5.2])
        para(f'lidarSLAM (kiến trúc Hess et al., 2016) chỉ dùng ghép quét, không dùng odometry; khi bật đóng vòng ATE giảm từ {TB["off"]["ate"]:.3f} xuống {TB["on"]["ate"]:.3f} m. Bản tự cài đặt đạt {sl["ate"]:.3f} m nhờ kết hợp thêm '
             'ràng buộc odometry, và quan trọng hơn cung cấp hiệp phương sai để vẽ biên 95 % – thứ lidarSLAM không trả về. ' +
             (f'Ngoài ra, một bản cài đặt độc lập bằng Python (cùng thuật toán, bộ sinh ngẫu nhiên khác) cho ATE SLAM = {PY["scenario1"]["slam"]["ate"]:.3f} m, cùng xu hướng.' if PY else ''))
    h2('4.7. Demo 3D robot hoạt động trong nhà xưởng')
    para('Chương trình demo_3d.py (Python + PyVista) đọc dữ liệu CSV do MATLAB xuất (quỹ đạo Simulink dày, keyframe, ước lượng SLAM online, hiệp phương sai, quét LiDAR, danh sách đóng vòng) '
         'và hiển thị **hai khung đồng bộ camera**: khung trái là nhà xưởng thật (ground truth) với robot chạy theo tư thế thật và tia LiDAR; khung phải là một **phòng trống** – robot chỉ có '
         'odometry và LiDAR, được đặt theo tư thế ước lượng của pose-graph SLAM, và bản đồ lưới chiếm chỗ hiện dần trên sàn (trắng: trống, xám: chưa biết, khối đen: ô vật cản P ≥ 0.65).')
    figure('demo_3d_live_mid.png', '4.11', 'Demo 3D hai khung – giữa hành trình: trái là ground truth, phải là phòng trống trong đó robot vừa chạy vừa tạo bản đồ', 16)
    para('Demo có hai chế độ: “live” – vừa chạy vừa tạo bản đồ theo tư thế SLAM online (Hình 4.11), và “after” – robot khảo sát hết vòng rồi mới dựng bản đồ từng scan theo quỹ đạo đã tối ưu. '
         'Ở cả hai chế độ, khi kết thúc bản đồ được dựng lại theo quỹ đạo SLAM tối ưu batch bằng đúng thuật toán log-odds của báo cáo, cho '
         f'Map Accuracy {sl["acc"]:.2f} % và Obstacle IoU {sl["iou"]:.1f} % – trùng với Bảng 4.1 (Hình 4.12). Demo chỉ phục vụ trực quan hóa; mọi số liệu đến từ MATLAB.')
    figure('demo_3d_live_final.png', '4.12', 'Demo 3D hai khung – kết thúc: bản đồ trong phòng trống dựng từ quỹ đạo SLAM tối ưu (xanh nhạt) khớp với nhà xưởng thật', 16)
    h2('4.8. Đánh giá mức độ đáp ứng yêu cầu và ưu – nhược điểm')
    minin = min(MC['slam']['in_x'], MC['slam']['in_y'], MC['slam']['in_t'])
    table('4.5', 'Đối chiếu kết quả với yêu cầu kỹ thuật (mục 3.1)', ['Yêu cầu', 'Ngưỡng', 'Kết quả', 'Đánh giá'],
          [['ATE RMSE của SLAM (σ_r = 3 cm)', '< 0.15 m', f'{sl["ate"]:.3f} m (MC: {MC["slam"]["ate_mean"]:.3f} m)', 'Đạt' if MC['slam']['ate_mean'] < 0.15 else 'Chưa đạt'],
           ['Map Accuracy', '> 90 %', f'{sl["acc"]:.2f} %', 'Đạt' if sl['acc'] > 90 else 'Chưa đạt'],
           ['% mẫu sai số trong ±2σ (MC)', '≥ 90 %', f'{minin:.1f} % (thấp nhất x, y, θ)', 'Đạt' if minin >= 90 else 'Chưa đạt'],
           ['Khử nhân đôi tường', 'Quan sát', 'Hình 4.6c: không còn tường kép', 'Đạt']], widths=[5.2, 2.2, 5.6, 3.0])
    para('**Ưu điểm:**', indent=False)
    bullet('Khử triệt để trôi tích lũy; độ chính xác cỡ vài cm trong phân xưởng 20 × 20 m; quỹ đạo tuân theo động học xe thực tế (Simulink).')
    bullet('Cho cả độ tin cậy (Σ = H⁻¹): vẽ được biên 95 %, elip 95 % và kiểm định thống kê – quan trọng cho lập kế hoạch an toàn.')
    bullet('Ground Truth tự thiết kế giúp lượng hóa tuyệt đối sai số thay vì đánh giá cảm tính.')
    para('**Nhược điểm và hạn chế:**', indent=False)
    bullet(orig('Mô phỏng hiện tại giả thiết môi trường nhà xưởng hoàn toàn tĩnh'))
    bullet(orig('Khi môi trường có các hành lang dài hoàn toàn trơn nhẵn'))
    bullet('Hiệp phương sai ICP theo xấp xỉ Hessian chưa hiệu chỉnh: có thể thận trọng ở nhiễu nhỏ và quá tự tin ở nhiễu lớn (mục 4.5).')
    bullet('Σ = H⁻¹ dạng đặc tốn O(N³); bản đồ lớn cần covariance recovery từ phân rã Cholesky thưa. Phát hiện đóng vòng dựa vào khoảng cách ước lượng nên phụ thuộc mức trôi hiện tại.')

    # ======================================================== CHƯƠNG 5
    h1('CHƯƠNG 5. KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN')
    h2('5.1. Kết luận')
    para('Đề tài đã hoàn thành các mục tiêu đề ra:')
    bullet('MT1: hệ thống hóa lý thuyết từ dead reckoning – lan truyền hiệp phương sai – biên 95 % (slide Ch.4) đến bản đồ lưới, front-end LiDAR, tối ưu đồ thị tư thế và thước đo SLAM; '
           'chỉ ra và kiểm chứng quan hệ Σ = H⁻¹ ↔ P của EKF.')
    bullet('MT2–MT3: xây dựng nhà xưởng chuẩn, sinh quỹ đạo bằng mô hình Simulink sl_drivepoint của sách, mô phỏng cảm biến và tự cài đặt đầy đủ pose-graph SLAM trên MATLAB; demo 3D bằng Python.')
    bullet(f'MT4: pose-graph SLAM đạt ATE {sl["ate"]:.3f} m, giảm {pct(dr["ate"], sl["ate"]):.1f} % so với dead reckoning và {pct(nl["ate"], sl["ate"]):.1f} % so với không đóng vòng; Map Accuracy {sl["acc"]:.2f} %, IoU {sl["iou"]:.1f} %. '
           'Biên ±2σ của SLAM hẹp hơn nhiều và co lại khi đóng vòng.' + (f' lidarSLAM đạt {TB["on"]["ate"]:.3f} m nhưng không có hiệp phương sai.' if TB else ''))
    bullet(f'MT5: qua {MC["M"]} lần Monte Carlo, DR có ANEES {MC["dr"]["anees"]:.2f}, SLAM {MC["slam"]["anees"]:.2f}, ≥ {minin:.0f} % mẫu SLAM nằm trong biên 95 %; hệ thống bền vững tới σ_r = 8 cm.')
    para('**Kết luận kỹ thuật chính:** kết hợp odometry, ghép quét LiDAR và đóng vòng trong một đồ thị tư thế cho phép vừa đạt độ chính xác cỡ centimet, vừa cung cấp mức độ tin cậy có kiểm chứng thống kê – điều kiện cần cho điều hướng an toàn của AMR.')
    h2('5.2. Hướng phát triển')
    bullet(orig('1. Kết hợp đa cảm biến').lstrip('1. ')); bullet(orig('2. Tích hợp lọc đóng vòng sai tự động').lstrip('2. '))
    bullet(orig('3. Ứng dụng bản đồ vào Lập kế hoạch đường đi').lstrip('3. '))
    bullet('Hiệu chỉnh hiệp phương sai ICP (Censi, 2007) để ANEES tiến về 2 trên mọi mức nhiễu; dùng bộ giải thưa (Cholesky, iSAM) cho bản đồ lớn.')
    bullet('Chuyển demo 3D sang ROS 2/Gazebo với LiDAR mô phỏng vật lý và kiểm chứng trên dữ liệu thật.')

    # ======================================================== TÀI LIỆU THAM KHẢO
    h1('TÀI LIỆU THAM KHẢO')
    refs = ['[1] P. Corke, W. Jachimczyk, and R. Pillat, Robotics, Vision and Control: Fundamental Algorithms in MATLAB, 3rd ed. Cham, Switzerland: Springer, 2023 (ch. 4, 6; App. E, F, G).',
            '[2] Bài giảng học phần Kỹ thuật Robot, “Chapter 4. Localization and Mapping – Part 1: Kalman filter; Part 2: Localization and Mapping”, 2025.',
            '[3] A. Elfes, “Using occupancy grids for mobile robot perception and navigation,” Computer, vol. 22, no. 6, pp. 46–57, Jun. 1989.',
            '[4] G. Grisetti, R. Kümmerle, C. Stachniss, and W. Burgard, “A tutorial on graph-based SLAM,” IEEE Intelligent Transportation Systems Magazine, vol. 2, no. 4, pp. 31–43, 2010.',
            '[5] R. Kümmerle et al., “On measuring the accuracy of SLAM algorithms,” Autonomous Robots, vol. 27, no. 4, pp. 387–407, 2009.',
            '[6] W. Hess, D. Kohler, H. Rapp, and D. Andor, “Real-time loop closure in 2D LIDAR SLAM,” in Proc. IEEE ICRA, Stockholm, 2016, pp. 1271–1278.',
            '[7] A. Censi, “An accurate closed-form estimate of ICP’s covariance,” in Proc. IEEE ICRA, Rome, 2007, pp. 3167–3172.',
            '[8] Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, Estimation with Applications to Tracking and Navigation. New York: Wiley, 2001.',
            '[9] P. Corke et al., “RVC3-MATLAB: Robotics, Vision & Control Toolbox (sl_drivepoint.slx, EKF.m),” 2023. [Online]. Available: https://github.com/petercorke/RVC3-MATLAB',
            '[10] MathWorks, “Implement Simultaneous Localization And Mapping (SLAM) with Lidar Scans,” Navigation Toolbox Documentation.']
    for t in refs:
        p = para(t, indent=False); p.paragraph_format.left_indent = Cm(0.8); p.paragraph_format.first_line_indent = Cm(-0.8)

    # ======================================================== PHỤ LỤC
    h1('PHỤ LỤC')
    h2('Phụ lục A. Cấu trúc mã nguồn và cách chạy')
    for t in ['matlab/main_pose_graph_slam.m – chương trình chính (Simulink → SLAM → đánh giá → hình → JSON/CSV → animation 2D).',
              'python/demo_3d.py – demo 3D PyVista phát lại kết quả MATLAB (pip install pyvista; python demo_3d.py [--follow] [--save]).',
              'python/build_report.py – dựng lại 2 bản Word từ kết quả MATLAB.',
              'Thứ tự chạy: MATLAB >> main_pose_graph_slam  →  python demo_3d.py  →  python build_report.py.']:
        bullet(t)
    if EXT:
        h2('Phụ lục C. Bảng tổng hợp: lý thuyết → nguồn → mã nguồn')
        table('C.1', 'Ánh xạ từng thành phần lý thuyết tới nguồn trích dẫn và hàm cài đặt', ['Thành phần', 'Nguồn (trang/mục/công thức)', 'Hàm trong main_pose_graph_slam.m'],
              [['Mô hình xe đạp, điều khiển tới điểm', 'Corke 2023, (4.4), (4.6)–(4.7), Hình 4.7, tr.132–135', 'mục 2 (sim sl_drivepoint)'],
               ['Mô hình odometry, nhiễu V', 'Slide Ch.4 p2 tr.11–13; Corke (6.1)–(6.2), tr.219–220', 'runSLAM: odo, oplus, relpose'],
               ['EKF dự đoán, Jacobian Fx, Fv', 'Slide tr.14–17; Corke (6.3)–(6.7), tr.222–223', 'deadReckoning'],
               ['Biên ±2σ = 95 %', 'Slide Ch.4 p2 tr.19; p1 tr.14, 96–100', 'boundPlot, metrics'],
               ['Elip 95 %, χ²₂ = 5.991', 'Slide p2 tr.23; Corke tr.224–225, App. G', 'ellipse95'],
               ['Occupancy grid, log-odds', 'Elfes 1989 tr.47–49; Hess 2016 §IV.B; Corke §6.3', 'buildMap, mapMetrics'],
               ['ICP điểm–đường, Σ_z', 'Censi 2007; Corke §6.3.2, App. F.2', 'icpPL, scanNormals'],
               ['Sai số cạnh e_ij', 'Grisetti (3)–(5), Phụ lục; Corke (6.33)', 'edgeJac'],
               ['Jacobian A, B; H Δx = −b', 'Grisetti (6)–(19); Corke tr.258–261, App. F.2.4', 'optimizePoseGraph, blocks'],
               ['Σ = H⁻¹ (Laplace)', 'Grisetti §IV; mục 2.5.3', 'margCov'],
               ['ATE, RPE', 'Kümmerle 2009 §3', 'metrics, rpe'],
               ['NEES, ANEES, biên χ²', 'Bar-Shalom 2001 §5.4', 'metrics, chi2q (KB2)'],
               ['lidarSLAM (submap, BnB)', 'Hess 2016 §IV–V; Corke §6.4.2 tr.263–266', 'mục 7 (KB4)']], widths=[4.6, 6.4, 5.0], size=10)
    h2('Phụ lục B. Mã nguồn MATLAB chính (main_pose_graph_slam.m)')
    for line in open(MAIN_M, encoding='utf-8').read().splitlines():
        p = d.add_paragraph(); p.paragraph_format.first_line_indent = Cm(0); p.paragraph_format.line_spacing = 1.0
        p.paragraph_format.space_after = Pt(0); p.alignment = WD_ALIGN_PARAGRAPH.LEFT
        r = p.add_run(line if line else ' '); r.font.name = 'Consolas'; r.font.size = Pt(8)
        r._element.get_or_add_rPr().get_or_add_rFonts().set(qn('w:hAnsi'), 'Consolas')

    # ---- danh mục hình/bảng
    for anchor, items, pre in ((FIG_ANCHOR, FIGN, 'Hình'), (TAB_ANCHOR, TABN, 'Bảng')):
        for k_, cap in items.items():
            np_ = OxmlElement('w:p'); anchor._p.addprevious(np_); pp = Paragraph(np_, anchor._parent)
            pp.paragraph_format.first_line_indent = Cm(0); pp.add_run(f'{pre} {k_}: ').bold = True; pp.add_run(cap)
    # ---- số trang + tự cập nhật trường + sửa schema
    fp = sec.footer.paragraphs[0] if sec.footer.paragraphs else sec.footer.add_paragraph(); fp.text = ''
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER; field(fp, 'PAGE', '1')
    st = d.settings.element; uf = OxmlElement('w:updateFields'); uf.set(qn('w:val'), 'true')
    after = ['w:hdrShapeDefaults', 'w:footnotePr', 'w:endnotePr', 'w:compat', 'w:docVars', 'w:rsids', 'm:mathPr', 'w:attachedSchema', 'w:themeFontLang',
             'w:clrSchemeMapping', 'w:doNotIncludeSubdocsInStats', 'w:doNotAutoCompressPictures', 'w:forceUpgrade', 'w:captions', 'w:readModeInkLockDown',
             'w:smartTagType', 'w:shapeDefaults', 'w:doNotEmbedSmartTags', 'w:decimalSymbol', 'w:listSeparator']
    nxt = next((st.find(qn(t)) for t in after if st.find(qn(t)) is not None), None)
    (nxt.addprevious(uf) if nxt is not None else st.append(uf))
    pm = sec._sectPr.find(qn('w:pgMar'))
    if pm is not None and pm.get(qn('w:gutter')) is None:
        pm.set(qn('w:gutter'), '0')
    d.save(out_path)
    print(f'Đã ghi {"BẢN MỞ RỘNG" if EXT else "BẢN CHÍNH THỨC"} -> {out_path}  ({len(FIGN)} hình, {len(TABN)} bảng, {EXTN[0]} khung mở rộng)')


build(False, DOCX)
build(True, EXTENDED)

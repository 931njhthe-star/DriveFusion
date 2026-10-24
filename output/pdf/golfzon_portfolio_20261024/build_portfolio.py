"""Build only the new Golfzon portfolio artifacts; read source data without editing it."""
from __future__ import annotations
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
from statistics import mean, median

from PySide6.QtCore import QMarginsF, QRectF, QSize, Qt, QUrl
from PySide6.QtGui import (QColor, QFont, QGuiApplication, QImage, QPageLayout,
    QPageSize, QPainter, QPdfWriter, QPen, QTextDocument, QTextOption)
from PySide6.QtPdf import QPdfDocument

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
WORKFLOW = ROOT / 'data/recordings/workflow_20261023_203600_8af7ab2a'
RESULTS = WORKFLOW / 'experiment_results'
PAPER = RESULTS / 'paper_radar_assisted_distance_20261024'
PREVIEW = ROOT / 'tmp/pdfs/golfzon_portfolio_20261024'
METHODS = ('camera_tracking', 'vision_radar_fusion', 'radar_tracking_fusion')
PAIRS = ((0,1),(1,2),(0,2))
WEATHER = [('ClearNoon','맑음 낮'),('ClearSunset','맑음 저녁'),('ClearNight','맑음 밤'),
    ('CloudyNoon','흐림 낮'),('CloudySunset','흐림 저녁'),('CloudyNight','흐림 밤'),
    ('HardRainNoon','강한 비 낮'),('HardRainSunset','강한 비 저녁'),('HardRainNight','강한 비 밤')]
BANDS = ['0~3m','3~5m','5~10m','10~15m','15~20m','20~30m','30m 이상']
SOURCE_PATHS = {
 'S01': ('보정 예측 437건의 원 분석', RESULTS/'radar_corrected_prediction_paired_analysis_20261023.md'),
 'S02': ('437·242·195건 통합 분석', RESULTS/'radar_corrected_all_groups_integrated_analysis_20261024.md'),
 'S03': ('통합 평가 CSV', RESULTS/'position_errors_weather_combined_all.csv'),
 'S04': ('논문 초안과 시뮬레이터 근거', PAPER/'manuscript.md'),
 'S05': ('현재 구현 기술 문서', ROOT/'docs/DETECTION_TRACKING_EVALUATION_TECHNICAL.md'),
 'S06': ('수집·상태·기록 코드', ROOT/'collector/live.py'),
 'S07': ('작업자·C ABI Python 브리지', ROOT/'monitor/qt/yolo_worker.py'),
 'S08': ('클래스 기반 탐욕 매칭 추적기', ROOT/'monitor/qt/tracker.py'),
 'S09': ('좌표·거리·품질 게이트', ROOT/'monitor/qt/positioning.py'),
 'S10': ('세션 평가와 가시성', ROOT/'monitor/qt/experiment_evaluator.py'),
 'S11': ('Qt UI와 워크플로', ROOT/'monitor/qt/app.py'),
 'S12': ('외부 TensorRTx CMake 빌드 기록', ROOT/'docs/PHASE_1_5_CMAKE.md'),
 'S13': ('외부 C++ 추론 구현', Path('C:/ai/tensorrtx/yolo11/src/detector.cpp')),
 'S14': ('레이더 보정·검증 테스트', ROOT/'tests/test_radar_track_correction.py'),
 'S15': ('WSL TCP 클라이언트', ROOT/'core/yolo11_tcp_detector.py'),
}
EXTRA_PATHS = [ROOT/'docs/PROJECT_OVERVIEW.md', ROOT/'docs/PORTFOLIO_PLAN.md',
    ROOT/'docs/RADAR_TRACK_CORRECTION.md',ROOT/'core/yolo11_detector.py',
    ROOT/'monitor/qt/visibility.py',ROOT/'monitor/qt/radar_track_correction.py',
    ROOT/'monitor/qt/weather_workflow.py',ROOT/'monitor/qt/weather_workflow_evaluator.py',
    ROOT/'tests/test_yolo11_tcp_detector.py',Path('C:/ai/tensorrtx/yolo11/CMakeLists.txt'),
    Path('C:/ai/tensorrtx/yolo11/include/detector_c.h'),Path('C:/ai/tensorrtx/yolo11/src/detector_c.cpp')]

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def key(row):return row['session'],row['frame'],row['gt_actor_id']
def truth(v):return v.lower()=='true'
def valid(v):
    try:return math.isfinite(float(v))
    except (ValueError,TypeError):return False

def stats(rows):
    values=[]
    for method in METHODS:
        es=[float(r[method]['tracking_distance_error_signed_m']) for r in rows]
        values.append(dict(N=len(es),MAE=mean(map(abs,es)) if es else None,
            RMSE=math.sqrt(mean(e*e for e in es)) if es else None,
            bias=mean(es) if es else None,median=median(map(abs,es)) if es else None))
    pairs=[]
    for a,b in PAIRS:
        better=worse=equal=0
        for r in rows:
            d=abs(float(r[METHODS[a]]['tracking_error_m']))-abs(float(r[METHODS[b]]['tracking_error_m']))
            if d>1e-9:better+=1
            elif d < -1e-9:worse+=1
            else:equal+=1
        pairs.append(dict(pair=f'{a+1}→{b+1}',
            MAE_delta=values[a]['MAE']-values[b]['MAE'] if rows else None,
            MAE_pct=(values[a]['MAE']-values[b]['MAE'])/values[a]['MAE']*100 if rows and values[a]['MAE'] else None,
            RMSE_delta=values[a]['RMSE']-values[b]['RMSE'] if rows else None,
            RMSE_pct=(values[a]['RMSE']-values[b]['RMSE'])/values[a]['RMSE']*100 if rows and values[a]['RMSE'] else None,
            better=better,worse=worse,equal=equal))
    return dict(N=len(rows),methods=values,pairs=pairs)

def verify():
    hashes={str(p):digest(p) for p in dict.fromkeys([x[1] for x in SOURCE_PATHS.values()]+EXTRA_PATHS)}
    chosen={};total=0
    with SOURCE_PATHS['S03'][1].open(encoding='utf-8-sig',newline='') as f:
        for r in csv.DictReader(f):
            total+=1
            if r['method']==METHODS[2] and truth(r['radar_corrected']) and not truth(r['observed']) and valid(r['tracking_error_m']):
                k=key(r);assert k not in chosen,'duplicate M3 key';chosen[k]={METHODS[2]:r}
    with SOURCE_PATHS['S03'][1].open(encoding='utf-8-sig',newline='') as f:
        for r in csv.DictReader(f):
            k=key(r)
            if k in chosen and r['method'] in METHODS[:2]:
                assert r['method'] not in chosen[k],'duplicate baseline key'
                chosen[k][r['method']]=r
    rows=list(chosen.values());assert len(rows)==437
    for r in rows:
        assert len(r)==3
        gt=float(r[METHODS[2]]['gt_distance_m'])
        for method in METHODS:
            x=r[method];assert valid(x['tracking_error_m']) and valid(x['tracking_distance_error_signed_m'])
            assert abs(float(x['gt_distance_m'])-gt)<1e-9
            assert abs(float(x['estimated_distance_m'])-gt-float(x['tracking_distance_error_signed_m']))<1e-8
            assert abs(abs(float(x['tracking_distance_error_signed_m']))-float(x['tracking_error_m']))<1e-8
    B=[r for r in rows if truth(r[METHODS[2]]['radar_used'])]
    C=[r for r in rows if not truth(r[METHODS[2]]['radar_used'])]
    assert len(B)==242 and len(C)==195 and len(B)+len(C)==len(rows)
    assert all(r[METHODS[2]]['position_source']=='radar_near_cluster' for r in B)
    groups={n:dict(overall=stats(rs),weather={w:stats([r for r in rs if r[METHODS[2]]['weather_preset']==w]) for w,_ in WEATHER},
        bands=[stats([r for r in rs if (0 if float(r[METHODS[2]]['gt_distance_m'])<3 else 1 if float(r[METHODS[2]]['gt_distance_m'])<5 else 2 if float(r[METHODS[2]]['gt_distance_m'])<10 else 3 if float(r[METHODS[2]]['gt_distance_m'])<15 else 4 if float(r[METHODS[2]]['gt_distance_m'])<20 else 5 if float(r[METHODS[2]]['gt_distance_m'])<30 else 6)==i]) for i in range(7)]) for n,rs in [('A',rows),('B',B),('C',C)]}
    historical=json.loads((PAPER/'data_snapshot.json').read_text(encoding='utf-8'))
    # The snapshot is only a cross-check; all portfolio tables derive from CSV above.
    previous=historical['populations']
    checked=0
    for n in groups:
        slices=[(groups[n]['overall'],previous[n]['overall'])]
        slices += [(groups[n]['weather'][w],previous[n]['weather'][w]) for w,_ in WEATHER]
        slices += [(groups[n]['bands'][i],previous[n]['bands'][str(i)]) for i in range(7)]
        for current,old in slices:
            assert current['N']==old['N']
            if not current['N']:continue
            for i,m in enumerate(METHODS):
                for metric in ('MAE','RMSE'):
                    assert abs(current['methods'][i][metric]-old['methods'][m][metric])<1e-9
                    checked+=1
    sessions=[]
    for p in sorted(WORKFLOW.glob('weather_*/*/session.json')):
        x=json.loads(p.read_text(encoding='utf-8'))
        sessions.append(dict(path=str(p),planned=x.get('planned_duration_seconds'),retention=x.get('tracking_max_lost_frames'),radar_correction=x.get('radar_track_correction_enabled')))
    assert len(sessions)==9 and all(s['planned']==120 and s['retention']==10 and s['radar_correction'] is True for s in sessions)
    reasons={}
    for r in C:
        reason=r[METHODS[2]]['distance_rejection_reason'];reasons[reason]=reasons.get(reason,0)+1
    assert sorted(reasons.values())==[12,53,130]
    return dict(csv_rows=total,corrected_candidates=437,paired=437,excluded=0,groups=groups,snapshot_metric_checks=checked,
        fallback_reasons=reasons,sessions=sessions,source_sha256=hashes,
        provenance='Recomputed from existing CSV, paired by session/frame/gt_actor_id. No YOLO/CARLA replay rerun.')

def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def f(v,n=3):return f'{v:.{n}f}' if v is not None else '측정 불가'
def source_link(p):return os.path.relpath(p,HERE).replace('\\','/')

def expand(verified):
    A=verified['groups']['A']['overall'];gs=verified['groups']
    inserts={
      'TABLE_A':table(['방법','동일 N','MAE m','RMSE m','편향 m','절대오차 중앙값 m'],[
          [f'M{i+1}',x['N'],f(x['MAE']),f(x['RMSE']),f"{x['bias']:+.3f}",f(x['median'])] for i,x in enumerate(A['methods'])]),
      'TABLE_PAIR':table(['비교','MAE 감소 m','MAE 개선율','RMSE 감소 m','RMSE 개선율'],[
          [p['pair'],f(p['MAE_delta']),f(p['MAE_pct'],1)+'%',f(p['RMSE_delta']),f(p['RMSE_pct'],1)+'%'] for p in A['pairs']]),
      'TABLE_GROUPS':table(['최종 출처 집단','N','M1 MAE/RMSE m','M2 MAE/RMSE m','M3 MAE/RMSE m'],[
          [label,gs[n]['overall']['N']]+[f(x['MAE'])+' / '+f(x['RMSE']) for x in gs[n]['overall']['methods']] for n,label in [('A','A 전체 보정'),('B','B 레이더 선택'),('C','C 카메라 대체')]]),
      'TABLE_GROUP_PAIR':table(['집단','비교','MAE 감소 m / %','RMSE 감소 m / %'],[
          [n,p['pair'],f(p['MAE_delta'])+' / '+f(p['MAE_pct'],1)+'%',f(p['RMSE_delta'])+' / '+f(p['RMSE_pct'],1)+'%'] for n in ('B','C') for p in gs[n]['overall']['pairs']]),
      'TABLE_WEATHER':table(['날씨·시간','N','M1 MAE m','M2 MAE m','M3 MAE m','M1→M3 개선'],[
          [label,gs['A']['weather'][w]['N']]+[f(x['MAE']) for x in gs['A']['weather'][w]['methods']]+[f(gs['A']['weather'][w]['pairs'][2]['MAE_pct'],1)+'%'] for w,label in WEATHER]),
      'TABLE_BANDS':table(['GT 거리','N','M1 MAE m','M2 MAE m','M3 MAE m','M1→M3 개선'],[
          [label,gs['A']['bands'][i]['N']]+[f(x['MAE']) for x in gs['A']['bands'][i]['methods']]+[f(gs['A']['bands'][i]['pairs'][2]['MAE_pct'],1)+'%'] for i,label in enumerate(BANDS)]),
      'SOURCES':'\n\n'.join(f'[{n}] [{label}]({source_link(p)})' for n,(label,p) in SOURCE_PATHS.items())
    }
    for path in sorted(HERE.glob('0[123]_*.md')):
        text=path.read_text(encoding='utf-8')
        for k,v in inserts.items():text=text.replace('{{'+k+'}}',v)
        text=text.replace('연구보고서 12쪽','연구보고서 13쪽')
        assert '{{' not in text,path
        path.write_text(text,encoding='utf-8')

def diagram():
    assets=HERE/'assets';assets.mkdir(exist_ok=True)
    image=QImage(2400,1060,QImage.Format.Format_ARGB32);image.fill('white')
    p=QPainter(image);p.setRenderHint(QPainter.RenderHint.Antialiasing);p.scale(2,2)
    def box(x,y,w,h,title,lines,color):
        p.setPen(QPen(QColor('#C6D2DB'),1));p.setBrush(QColor(color));p.drawRoundedRect(QRectF(x,y,w,h),9,9)
        p.setPen(QColor('#162C3B'));p.setFont(QFont('Malgun Gothic',14,QFont.Weight.Bold));p.drawText(QRectF(x+15,y+14,w-30,34),Qt.AlignmentFlag.AlignLeft,title)
        p.setFont(QFont('Malgun Gothic',11))
        for j,line in enumerate(lines):p.drawText(QRectF(x+15,y+53+j*26,w-30,28),Qt.AlignmentFlag.AlignLeft,line)
    def arrow(x1,y1,x2,y2):
        p.setPen(QPen(QColor('#46657A'),2));p.drawLine(x1,y1,x2,y2)
        if x2>x1:p.drawLine(x2,y2,x2-9,y2-5);p.drawLine(x2,y2,x2-9,y2+5)
        elif y2>y1:p.drawLine(x2,y2,x2-5,y2-9);p.drawLine(x2,y2,x2+5,y2-9)
    box(15,15,310,140,'CARLA 센서와 수집',['RGB · 레이더 · GT · 마스크','frame / simulation / receive'], '#EFF5F9')
    box(385,15,360,140,'LiveState',['최신 영상 / frame별 레이더·GT','Lock + Condition / 최근 256프레임'], '#EFF5F9')
    box(805,15,375,140,'YoloWorker',['C++/CUDA YOLO11n DLL','영상 추적 / 레이더 보조 추적'], '#E7F2EF')
    arrow(325,85,385,85);arrow(745,85,805,85)
    box(15,235,310,155,'SessionRecorder',['레코드 4096 / 영상·마스크 각 32','별도 기록 스레드 / 누락 집계','JSONL · JPEG · PNG'], '#FFF4E8')
    box(385,235,360,155,'Qt 검증 화면',['A/B 비교 / 거리 출처 / 수신률','수동·시간 지정 녹화','날씨 워크플로 / 보고서 실행'], '#EFF5F9')
    box(805,235,375,155,'PositioningEngine',['같은 frame 좌표 변환·연관','거리 선택 / 보정·거부 사유','차체 간격 산출'], '#E7F2EF')
    arrow(170,155,170,235);arrow(565,155,565,235);arrow(990,155,990,235)
    box(15,440,1165,72,'저장 입력 재생 평가',['동일 session · frame · GT actor → 세 방법 공통 표본 → CSV · JSON · 조건별 / 통합 보고서'], '#F2F4F7')
    arrow(170,390,170,440);arrow(990,390,990,440)
    p.end();image.setDotsPerMeterX(11811);image.setDotsPerMeterY(11811);image.save(str(assets/'architecture.png'))
    shutil.copy2(PAPER/'assets/fig02_main_errors.png',assets/'main_errors.png')

def inline(s):
    s=html.escape(s)
    s=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',lambda m:f'<a href="{m[2]}">{m[1]}</a>',s)
    s=re.sub(r'\*\*([^*]+)\*\*',r'<b>\1</b>',s)
    s=re.sub(r'`([^`]+)`',r'<span style="font-family:Consolas;font-size:8.8pt">\1</span>',s)
    return s

def page_html(md):
    lines=md.strip().splitlines();out=[];para=[];i=0
    def flush():
        if para:out.append('<p>'+inline(' '.join(para))+'</p>');para.clear()
    while i<len(lines):
        s=lines[i].strip()
        if not s:flush();i+=1;continue
        if s.startswith('|'):
            flush();rows=[]
            while i<len(lines) and lines[i].strip().startswith('|'):
                cs=[c.strip() for c in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',c) for c in cs):rows.append(cs)
                i+=1
            out.append('<table width="100%" cellspacing="0" cellpadding="5">')
            for j,cells in enumerate(rows):
                tag='th' if j==0 else 'td';bg=' bgcolor="#E9EFF4"' if j==0 else ''
                out.append('<tr>'+''.join(f'<{tag}{bg}>{inline(c)}</{tag}>' for c in cells)+'</tr>')
            out.append('</table>');continue
        match=re.fullmatch(r'!\[([^\]]*)\]\(([^)]+)\)',s)
        if match:
            flush();im=QImage(str(HERE/match[2]));assert not im.isNull()
            w=655;h=round(w*im.height()/im.width());out.append(f'<p align="center" style="line-height:100%;margin-top:7px;margin-bottom:8px"><img src="{match[2]}" width="{w}" height="{h}" alt="{match[1]}"/></p>');i+=1;continue
        match=re.match(r'^(#{1,6}) (.+)',s)
        if match:flush();level=len(match[1]);out.append(f'<h{level}>{inline(match[2])}</h{level}>');i+=1;continue
        para.append(s);i+=1
    flush();return '\n'.join(out)

CSS="""body{color:#182F3F;font-family:'Malgun Gothic';font-size:10pt;}
p{margin-top:0px;margin-bottom:10px;line-height:140%;}
h1{font-size:23pt;line-height:125%;margin-top:0px;margin-bottom:22px;}
h2{font-size:16pt;margin-top:0px;margin-bottom:14px;}
h3{font-size:11.5pt;margin-top:15px;margin-bottom:8px;}
table{font-size:8.8pt;border-collapse:collapse;margin-top:8px;margin-bottom:12px;}
td,th{border:0.6px solid #CAD5DE;vertical-align:middle;padding:5px;}
th{font-weight:bold;}a{color:#176189;text-decoration:none;}"""

def render(path):
    md=path.read_text(encoding='utf-8')
    for dash in '\u2010\u2011\u2012\u2013\u2014\u2212':md=md.replace(dash,'-')
    pages=[page_html(s) for s in md.split('<!-- PAGE -->')]
    screen=CSS+"""html{background:#E8EDF2}body{margin:0}.toolbar{position:sticky;top:0;background:#183448;color:white;padding:14px 24px}.toolbar a{color:#DCEEFF;margin-right:22px}.page{box-sizing:border-box;width:210mm;min-height:297mm;padding:16mm;margin:20px auto;background:white;box-shadow:0 4px 22px #10253622}img{max-width:100%;height:auto}@media print{.toolbar{display:none}.page{margin:0;box-shadow:none;break-after:page;min-height:0;width:auto;padding:0}@page{size:A4;margin:16mm}}"""
    title=md.splitlines()[0].lstrip('# ')
    reader='<div class="toolbar"><a href="index.html">문서 목록</a><a href="'+path.stem+'.pdf">PDF</a><a href="'+path.name+'">편집 원본</a><button onclick="window.print()">인쇄</button></div>'
    reader+=''.join('<article class="page">'+x+'</article>' for x in pages)
    path.with_suffix('.html').write_text('<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>'+screen+'</style></head><body>'+reader+'</body></html>',encoding='utf-8')
    writer=QPdfWriter(str(path.with_suffix('.pdf')));writer.setResolution(96)
    writer.setPageLayout(QPageLayout(QPageSize(QPageSize.PageSizeId.A4),QPageLayout.Orientation.Portrait,QMarginsF(0,0,0,0),QPageLayout.Unit.Millimeter))
    writer.setTitle(title);writer.setCreator('CARLA RadarVision Portfolio')
    W,H=writer.width(),writer.height();margin=58;available=H-125;docs=[]
    for i,page in enumerate(pages):
        d=QTextDocument();d.setDefaultFont(QFont('Malgun Gothic',10));d.setDocumentMargin(0)
        option=d.defaultTextOption();option.setWrapMode(QTextOption.WrapMode.WrapAtWordBoundaryOrAnywhere);d.setDefaultTextOption(option)
        d.setBaseUrl(QUrl.fromLocalFile(str(HERE)+os.sep));d.setDefaultStyleSheet(CSS)
        d.setHtml('<html><body>'+page+'</body></html>');d.setTextWidth(W-2*margin);docs.append(d)
    overflow=[(i+1,round(d.size().height(),1)) for i,d in enumerate(docs) if d.size().height()>available]
    if overflow:raise RuntimeError(f'{path.name}: overflow {overflow}; available {available}; heights {[round(d.size().height()) for d in docs]}')
    p=QPainter(writer)
    for i,d in enumerate(docs):
        if i:writer.newPage()
        p.fillRect(QRectF(0,0,W,H),Qt.GlobalColor.white)
        p.save();p.translate(margin,57);d.drawContents(p,QRectF(0,0,W-2*margin,available));p.restore()
        p.setPen(QColor('#62788A'));p.setFont(QFont('Malgun Gothic',7.5))
        p.drawText(QRectF(margin,22,W-2*margin,20),Qt.AlignmentFlag.AlignLeft,'CARLA RadarVision  |  골프존 지원 기술 포트폴리오')
        p.drawText(QRectF(margin,H-42,W-2*margin,20),Qt.AlignmentFlag.AlignRight,f'{i+1} / {len(docs)}')
    p.end();pdf=QPdfDocument();assert pdf.load(str(path.with_suffix('.pdf')))==QPdfDocument.Error.None_;assert pdf.pageCount()==len(pages)
    target=PREVIEW/path.stem;target.mkdir(parents=True,exist_ok=True)
    for i in range(pdf.pageCount()):
        assert pdf.getAllText(i).text().strip(),f'Empty PDF page {i+1}'
        assert pdf.render(i,QSize(1191,1684)).save(str(target/f'page_{i+1:02}.png'))
    cols=3;tw,th=298,421;rows=math.ceil(len(pages)/cols)
    sheet=QImage(tw*cols,rows*(th+26),QImage.Format.Format_ARGB32);sheet.fill('#D8E2EA');q=QPainter(sheet)
    q.setPen(QColor('#152D3E'));q.setFont(QFont('Arial',11))
    for i in range(len(pages)):
        x=i%cols*tw;y=i//cols*(th+26);q.drawImage(QRectF(x,y,tw,th),QImage(str(target/f'page_{i+1:02}.png')));q.drawText(QRectF(x,y+th,tw,26),Qt.AlignmentFlag.AlignCenter,str(i+1))
    q.end();sheet.save(str(target/'contact_sheet.png'))
    return dict(file=path.with_suffix('.pdf').name,pages=len(pages),heights=[round(d.size().height(),1) for d in docs],available=available)

def evidence(verified):
    lines=['# 포트폴리오 근거 색인','', '작성 기준 2026-10-24. 채용공고는 사용자가 제공한 내용으로 매핑했으며 회사 매출·인원·설립일 등은 문서에서 주장하지 않았다.','',
        '## 자료 우선순위','', '현재 소스와 실제 실험 기록을 우선했다. PROJECT_OVERVIEW의 기준일은 2026-10-21이므로 이후 기능과 차이가 있다. PORTFOLIO_PLAN의 목표 구조를 완료 성과로 전환하지 않았다. 외부 TensorRTx는 제3자 기반이며 프로젝트 전체를 직접 작성한 C++/CUDA 신경망으로 표현하지 않았다.','',
        '## 주요 자료','']
    for n,(label,p) in SOURCE_PATHS.items():lines.extend([f'### {n} {label}',f'[{p.name}]({source_link(p)})',''])
    lines+=['## 추가 코드와 기록','']+[f'[{p.name}]({source_link(p)})\n' for p in EXTRA_PATHS]
    lines+=['## 재현 절차','', '`C:\\programming\\carla\\.venv\\Scripts\\python.exe output/pdf/golfzon_portfolio_20261024/build_portfolio.py`','',
        '기존 CSV를 두 번 읽어 M3 보정 예측을 선정한 뒤 M1·M2를 동일 키로 연결한다. 중복·유효 오차·GT 일치·오차 재계산·242/195 분할·메타데이터를 검사한다. YOLO 재추론 또는 CARLA 재생 실험을 새로 실행하지 않는다. 출력 대상은 이 포트폴리오 폴더와 tmp/pdfs/golfzon_portfolio_20261024의 미리보기뿐이다.','',
        '## 제출 전 개인 정보','', '성명·연락처·수행 기간·본인 담당 범위는 확인되지 않아 새로 만들지 않았다. 개인 이력과 맞는 정보를 별도 이력에 기재한다. 경력 5년, 원시 RF DSP, MCU/FPGA 경험을 이 프로젝트만으로 입증하지 않는다.']
    (HERE/'evidence_index.md').write_text('\n\n'.join(lines),encoding='utf-8')
    (HERE/'verification.json').write_text(json.dumps(verified,ensure_ascii=False,indent=2),encoding='utf-8')

def index(rendered):
    labels=[('센서 융합 알고리즘 연구보고서','알고리즘 설계 · 계측 기준 · 실험 · 실패 분석'),('실시간 소프트웨어 아키텍처 설계서','C++/CUDA 경계 · 동시성 · 기록 · Qt · 확장 설계'),('연구개발 성과 요약서','담당자가 먼저 읽을 핵심 결과와 직무 연결')]
    cards=[]
    for r,(title,subtitle) in zip(rendered,labels):
        stem=Path(r['file']).stem
        cards.append(f'<section><p class="eyebrow">{r["pages"]} PAGES</p><h2>{title}</h2><p>{subtitle}</p><p><a href="{stem}.pdf">PDF 열기</a><a href="{stem}.html">브라우저에서 읽기</a><a href="{stem}.md">편집 원본</a></p></section>')
    body='''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>골프존 지원 기술 포트폴리오</title><style>body{margin:0;background:#EEF3F7;color:#163043;font:16px/1.75 'Malgun Gothic',sans-serif}main{max-width:980px;margin:48px auto;padding:0 24px}h1{font-size:34px;line-height:1.4}section{background:white;padding:28px 32px;margin:22px 0;border:1px solid #D9E3EB;border-radius:10px}.eyebrow{letter-spacing:2px;font-size:12px;color:#57778F}h2{font-size:23px;margin:8px 0}a{display:inline-block;color:#12618B;margin-right:22px;text-decoration:none;font-weight:bold}.note{color:#506778;font-size:14px}</style></head><body><main><p class="eyebrow">CARLA RADARVISION · 2026.10.24</p><h1>골프존 소프트웨어 연구개발<br>지원 기술 포트폴리오</h1><p>센서 취득과 C++/CUDA 추론 모듈을 연결하고 Qt로 검증하며, 동일 기록에서 알고리즘 성능을 비교한 프로젝트.</p>'''
    body+=''.join(cards)+'<p class="note">기존 데이터·소스·분석 보고서는 보존했습니다. 수치는 레이더 보정 예측 437개 공통 표본의 결과이며 실환경·전체 미검출·골프공 계측의 일반 성능을 의미하지 않습니다.</p><p><a href="evidence_index.md">근거·구현 범위·재현 절차</a></p></main></body></html>'
    (HERE/'index.html').write_text(body,encoding='utf-8')

def main():
    app=QGuiApplication.instance() or QGuiApplication(sys.argv)
    if '--render-only' in sys.argv:
        verified=json.loads((HERE/'verification.json').read_text(encoding='utf-8'))
    else:
        verified=verify();expand(verified);evidence(verified)
    diagram()
    rendered=[render(p) for p in sorted(HERE.glob('0[123]_*.md'))]
    assert [r['pages'] for r in rendered]==[13,7,2]
    for path,expected in verified['source_sha256'].items():assert digest(Path(path))==expected,'source modified: '+path
    index(rendered)
    (HERE/'render_validation.json').write_text(json.dumps(rendered,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(documents=rendered,paired=verified['paired'],groups={n:v['overall']['N'] for n,v in verified['groups'].items()},source_files_unchanged=True),ensure_ascii=True))

if __name__=='__main__':main()

---
title: "פרמטרי סף יציבות — Platform Stability Monitor v2"
lang: he
mainfont: "Arial"
fontsize: 11pt
geometry: "top=3cm, bottom=3cm, left=2.5cm, right=2.5cm"
header-includes:
  - \usepackage{longtable}
  - \usepackage{booktabs}
  - \usepackage{xcolor}
  - \usepackage{colortbl}
  - \usepackage{array}
  - \renewcommand{\arraystretch}{2.4}
  - \setlength{\tabcolsep}{10pt}
  - \definecolor{rowgray}{RGB}{240,242,245}
  - \definecolor{headerblue}{RGB}{210,225,245}
  - \rowcolors{2}{rowgray}{white}
---

| פרמטר | ערך ברירת מחדל | משמעות |
|---|---|---|
| `window_s` | 1 שנייה | אורך חלון הזמן לניתוח יציבות — כמה שניות אחורה נבדקות הדגימות (טווח: 1–5 שניות, ~104 דגימות לשנייה) |
| `omega_stable_dps` | 20.0 °/s | סף מהירות זוויתית — חריגה מעל ערך זה בחלון נספרת כדגימת תנועה |
| `omega_instant_dps` | 45.0 °/s | סף מהירות זוויתית מיידית — חריגה מפעילה עיכוב כניסה ליציבות |
| `instant_hold_ms` | 250 ms | משך עיכוב לאחר חריגת מהירות מיידית — המערכת לא תכריז יציב בפרק זמן זה |
| `spike_accel_g` | 0.10 g | סף חריגת תאוצה — סטייה מ-1g מעל ערך זה נספרת כדגימת זעזוע |
| `eval_period_ms` | 200 ms | תדירות הערכת יציבות — כל כמה אלפיות שנייה מתבצעת הערכת המדדים המצטברים |
| `motion_samples_max` | 56 דגימות | מספר מקסימלי של דגימות תנועה מותרות בחלון — מעל זה: לא יציב |
| `shock_samples_max` | 3 דגימות | מספר מקסימלי של דגימות זעזוע מותרות בחלון — מעל זה: לא יציב |
| `clean_streak_needed` | 5 מחזורים | מספר מחזורי הערכה תקינים רצופים הנדרשים לניקוי תקלה פעילה (GYRO_RANGE, ACCEL_RANGE, TIMING, DIVERSE, DUAL_DIVERGE) |
| `diverse_mean_tol_g` | 0.1 g | סטייה מקסימלית מותרת של ממוצע תאוצת החלון מ-1g — בודק שהחיישן מוטה כנגד הכבידה ולא בנפילה חופשית או רטט |
| `spread_max_deg` | 7.5° | רדיוס פיזור מרבי סביב הצנטרואיד של הטיה בחלון — מעל זה הפלטפורמה נחשבת לא יציבה; מוגן ב-CRC-16 |
| `anchor_max_deg` | 8.0° | סטייה מקסימלית מנקודת העיגון — מונע סחיפה איטית שאינה מתגלה בחלון הפיזור; מוגן ב-CRC-16 |
| `gyro_range_fault_dps` | 248.0 °/s | תקרת מהירות זוויתית — חריגה מכריזה על FAULT_GYRO_RANGE (99% מ-250 °/s, טווח מלא של ASM330LHB בהגדרה הנוכחית) |
| `stale_fault_ticks` | 15 טיקים (~18 ms) | מספר קריאות עוקבות כושלות מהחיישן לפני הצהרת תקלת תקשורת — בקצב 833 Hz: 15 × 1.2 ms ≈ 18 ms |
| `dual_diverge_deg` | 5.0° | סטייה מקסימלית מותרת בין ערוץ A לערוץ B (roll או pitch בנפרד) — חריגה מצביעה על אי-התאמה בין שני החיישנים (FAULT_DUAL_DIVERGE) |

%对外提供的类的名字
\ProvidesClass{PaperReadingLog}[2020/07/26 PaperReading笔记类]
%继承的类
\LoadClass{article}
%%兼容中文
\RequirePackage{ctex}

%预存颜色
\RequirePackage{xcolor}
\definecolor{hyperlink}{HTML}{6699CC}%超链接的默认颜色
\definecolor{black}{HTML}{000000}
\definecolor{grey}{HTML}{404040}
\definecolor{red}{HTML}{FF3366}


%必须用xelatex来进行编译，fontspec包用于控制字体
\RequirePackage{fontspec}
\RequirePackage[colorlinks, linkcolor=black, urlcolor=hyperlink, bookmarks=false]{hyperref}

% 设置margin
\RequirePackage[margin=0.5in]{geometry}

% 设置多栏布局
\RequirePackage{multicol}

%使用绝对路径来指定要使用的字体
\newcommand{\FangzhengHeiti}
{\fontspec[ExternalLocation=C:/Users/64526/AppData/Local/Microsoft/Windows/Fonts/]{FZHTJW_0.TTF}}
\newcommand{\MicroYaHei}
{\fontspec[ExternalLocation=C:/Windows/Fonts/]{msyh.ttc}}
\newcommand{\Arial}
{\fontspec[ExternalLocation=C:/Windows/Fonts/]{arial.ttf}}

%定制段落之间的格式
\RequirePackage{titlesec}
\titleformat*{\section}{\Large\Arial\color{black}\bfseries}
\titleformat*{\subsection}{\large\Arial\color{black}\bfseries}
\titleformat*{\subsubsection}{\Arial\color{black}}
\titleformat*{\paragraph}{\bfseries\Arial\color{black}}
% \titlespacing*{\chapter} {0pt}{50pt}{40pt}
% \titlespacing*{\section} {0pt}{3.5ex plus 1ex minus .2ex}{2.3ex plus .2ex}
% \titlespacing*{\subsection} {0pt}{3.25ex plus 1ex minus .2ex}{1.5ex plus .2ex}
% \titlespacing*{\subsubsection}{0pt}{3.25ex plus 1ex minus .2ex}{1.5ex plus .2ex}
% \titlespacing*{\paragraph} {0pt}{3.25ex plus 1ex minus .2ex}{1em}
% \titlespacing*{\subparagraph} {parindent}{3.25ex plus 1ex minus .2ex}{1em}

\RequirePackage{graphicx}

\RequirePackage{enumitem} %枚举列表

%设置枚举列表和正文文本的间距
\setenumerate[1]{itemsep=0pt,partopsep=0pt,parsep=\parskip,topsep=0pt}
\setitemize[1]{itemsep=0pt,partopsep=0pt,parsep=\parskip,topsep=0pt}
\setdescription{itemsep=0pt,partopsep=0pt,parsep=\parskip,topsep=0pt}


%%处理论文的信息，参数分别为：论文名字、论文作者、链接地址、论文主题
\newcommand{\PaperInfo}[4]{
    \begin{flushleft}%段落左对齐
    \huge \MicroYaHei \color{hyperlink}{#1} %显示论文名称，字比较大
    
    \color{grey}\normalsize \Arial #2 #3%显式作者
    \end{flushleft}
}
%%处理演讲的信息，包含演讲者、演讲时间
\newcommand{\SpeechInfo}[2]{
    %\begin{flushleft}%段落左对齐
        \small \FangzhengHeiti \textcolor{grey}{演讲者：#1 \hfill 演讲时间：#2}
    %\end{flushleft}
}
\newcommand{\key}[1]{
    \color{red} #1 \color{black}
}
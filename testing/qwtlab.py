from qt import *
from qwt import *
from numpy import arange, linspace
import Numeric

"Plotting Tools"

# def linspace(xmin, xmax, N):
#    if N==1: return xmax
#    dx = (xmax-xmin)/(N-1)
#    return xmin + dx*arange(N)

def semilogy():
    _qwt_axes.setAxisOptions(QwtPlot.yLeft,
                             QwtAutoScale.Logarithmic)
def semilogx(on):
    l = {True: QwtAutoScale.Logarithmic, False: 0}[on]
    _qwt_axes.setAxisOptions(QwtPlot.xBottom, l)

def axes(a):
    global _qwt_axes
    _qwt_axes = a
    _qwt_axes.canvas().setFocusIndicator(QwtPlotCanvas.NoFocusIndicator)

def axis(x0, x1, y0, y1):
    _qwt_axes.setAxisScale(QwtPlot.yLeft, y0, y1)
    _qwt_axes.setAxisScale(QwtPlot.xBottom, x0, x1)

def yaxis(y0, y1):
    _qwt_axes.setAxisScale(QwtPlot.yLeft, y0, y1)

def xaxis(x0, x1):
    _qwt_axes.setAxisScale(QwtPlot.xBottom, x0, x1)

def xlabel(l):
    _qwt_axes.setAxisTitle(QwtPlot.xBottom, l)

def ylabel(l):
    _qwt_axes.setAxisTitle(QwtPlot.yLeft, l)

def drawnow():
    _qwt_axes.replot()

def resetzoom():
    global _qwt_zoom
    _qwt_zoom = QwtPlotZoomer(QwtPlot.xBottom,
                              QwtPlot.yLeft,
                              QwtPicker.DragSelection,
                              QwtPicker.AlwaysOff,
                              _qwt_axes.canvas())

    _qwt_zoom.setRubberBandPen(QPen(Qt.gray))

def plot(*curves):
    commands = {
        "r": lambda c: c.setPen(QPen(Qt.red)),
        "b": lambda c: c.setPen(QPen(Qt.blue)),
        "g": lambda c: c.setPen(QPen(Qt.green)),
        "k": lambda c: c.setPen(QPen(Qt.black)),
        "c": lambda c: c.setPen(QPen(Qt.cyan)),
        "y": lambda c: c.setPen(QPen(Qt.yellow)),
        ":": lambda c: c.pen().setStyle(Qt.DotLine)
    }

    newplot = not hasattr(_qwt_axes, "cs")
    if newplot:
        _qwt_axes.cs = []
        for n in range(len(curves)):
            qc = QwtPlotCurve(_qwt_axes)
            _qwt_axes.cs.append(qc)
            _qwt_axes.insertCurve(qc)

    for ((x, y, fmt), qc) in zip(curves, _qwt_axes.cs):
        for c in fmt:
            commands[c](qc)
        x = Numeric.array(x)
        y = Numeric.array(y)
        qc.setData(x, y)

    _qwt_axes.replot()

    if newplot:
        # add zoomer
        global _qwt_zoom
        _qwt_zoom = QwtPlotZoomer(QwtPlot.xBottom,
                                  QwtPlot.yLeft,
                                  QwtPicker.DragSelection,
                                  QwtPicker.AlwaysOff,
                                  _qwt_axes.canvas())

        _qwt_zoom.setRubberBandPen(QPen(Qt.gray))



def plotarray2(ar):
    colours = 'kbgr'

    t = arange(ar.shape[1])
    plot(*[(t, a, c) for a, c in zip(ar, colours)])


__all__ = [
    "axes", "axis", "yaxis", "xaxis", "xlabel", "ylabel",
    "plot", "semilogy", "semilogx", "drawnow", "resetzoom", "linspace"]

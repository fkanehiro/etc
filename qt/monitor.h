#include <QMainWindow>

class CanvasWidget;
class QTableView;
class QLabel;
class QPushButton;
class QSlider;
class GLscene;
class Simulator;
class SimulationThread;
class QBasicTimer;

class monitor : public QMainWindow
{
    Q_OBJECT

public:
    monitor();
    ~monitor();
    void keyPressEvent(QKeyEvent *pEvent);
    void keyReleaseEvent(QKeyEvent *pEvent);
    void timerEvent(QTimerEvent *e);
    void setSimulator(Simulator *i_simulator) { simulator = i_simulator; }
private:
    void createActions();
    void createMenus();
    void createToolBar();
    void createStatusBar();
    int getKeyPressingAccel(QKeyEvent *event);
    
    QStatusBar* statusBar;
    QSlider *playJustSlider;
    QToolBar *playToolBar;
    QLabel *worldTimeLabel;
    QPushButton *playButton;
    QPushButton *slowerButton;
    QPushButton *fasterButton;
    QPushButton *startButton;
    QBasicTimer *timer;
    
    int keyPressingCount;
    GLscene *scene;
    Simulator *simulator;
    SimulationThread *simulationThread;

    CanvasWidget *canvas;
    QTableView *table;
                     
private slots:
    void play();
    void playJust(int value);
    void faster();
    void slower();
};

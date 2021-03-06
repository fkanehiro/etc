#include <QBasicTimer>
#include <QSplitter>
#include <QStatusBar>
#include <QTableView>
#include <QLabel>
#include <QToolBar>
#include <QPushButton>
#include <QKeyEvent>
#include <QTime>
#include <iostream>
#include "SimulationThread.h"
#include "GLmodel.h"
#include "monitor.h"
#include "canvaswidget.h"
#include "mymodel.h"

monitor::monitor() : 
    scene(GLscene::getInstance())
{
    QSplitter *pSplitter = new QSplitter(this);
    pSplitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setCentralWidget(pSplitter);

    window()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    canvas = new CanvasWidget(pSplitter);
    canvas->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pSplitter->addWidget(canvas);

#if 0
    table = new QTableView(pSplitter);
    MyModel myModel(0);
    table->setModel( &myModel );
    table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pSplitter->addWidget(table);

    pSplitter->setStretchFactor(pSplitter->indexOf(canvas), QSizePolicy::Ignored);
#endif

    //createActions();
    //createMenus();
    createToolBar();
    //createStatusBar();

    simulationThread = NULL;

    timer = new QBasicTimer;
    timer->start(33, this);
}

monitor::~monitor()
{
    delete canvas;
    //RTC::Manager::instance().shutdown();
}

void monitor::createActions()
{
}

void monitor::createMenus()
{
}

void monitor::createToolBar()
{
    playToolBar = addToolBar(tr("Play"));

#if 0
    startButton = new QPushButton(tr("&start"));
    playToolBar->addWidget(startButton);
    connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
#endif

    //worldTimeLabel = new QLabel();
    //worldTimeLabel->setText(tr("0.000"));
    //playToolBar->addWidget(worldTimeLabel);

    slowerButton = new QPushButton(tr("&<<"));
    playToolBar->addWidget(slowerButton);

    playButton = new QPushButton(tr("&Play"));
    playToolBar->addWidget(playButton);
    //playButton->setEnabled(false);

    fasterButton = new QPushButton(tr("&>>"));
    playToolBar->addWidget(fasterButton);

    playJustSlider = new QSlider(Qt::Horizontal, this);
    playJustSlider->setMaximum(0);
    //playJustSlider->setEnabled(false);
    playToolBar->addWidget(playJustSlider);
    //playJustSlider->setTickPosition(QSlider::TicksAbove);
    //playJustSlider->setTickInterval(10);

    connect(slowerButton, SIGNAL(clicked()), this, SLOT(slower()));
    connect(playButton, SIGNAL(clicked()), this, SLOT(play()));
    connect(fasterButton, SIGNAL(clicked()), this, SLOT(faster()));
    connect(playJustSlider, SIGNAL(valueChanged(int)), this, SLOT(playJust(int)));
}

void monitor::createStatusBar()
{
    statusBar = new QStatusBar(this);
    statusBar->setEnabled(true);
    setStatusBar(statusBar);
}

void monitor::keyPressEvent(QKeyEvent *pEvent) 
{
    if( canvas->hasFocus() &&
        playJustSlider->isEnabled() )
    {
        int nStep = playJustSlider->singleStep();
        int nPos = playJustSlider->sliderPosition();
        switch( pEvent->key() )
        { 
        case Qt::Key_Right:
        case Qt::Key_Up:
            nStep += getKeyPressingAccel(pEvent);
            nPos += nStep;
            playJustSlider->setValue(nPos);
            break;
        case Qt::Key_Left:
        case Qt::Key_Down:
            nStep += getKeyPressingAccel(pEvent);
            nPos -= nStep;
            playJustSlider->setValue(nPos);
            break;
        default:
            break;
        }
    }
    QWidget::keyPressEvent(pEvent);
}

void monitor::keyReleaseEvent(QKeyEvent *pEvent)
{
    switch(pEvent->key())
    {
    case Qt::Key_Right:
    case Qt::Key_Up:
    case Qt::Key_Left:
    case Qt::Key_Down:
        if(! pEvent->isAutoRepeat() )
            keyPressingCount = 0;
        break;
    default:
        break;
    }
}

int monitor::getKeyPressingAccel(QKeyEvent *event)
{
    static QTime postTime = QTime().currentTime();
    QTime currentTime = QTime().currentTime();

    if(!event->isAutoRepeat())
        postTime = currentTime;
    int mSec = postTime.msecsTo(currentTime);

    if( mSec > 100)
    {
        ++keyPressingCount;
        postTime = currentTime;
    }else if (mSec < 0){
        postTime = currentTime;
    }

    return keyPressingCount;
}

void monitor::play()
{
    if (playButton->text() == tr("&Play")){ // start
        playButton->setText(tr("&Stop"));
    }else{
        playButton->setText(tr("&Play"));
    }
    scene->play();
}

void monitor::playJust(int value)
{
    //std::cout << "playJust(" << value << ")" << std::endl;
    scene->setLogPosition(value);
    //double tm = scene->time();
    //worldTimeLabel->setText(tr("%1").arg(tm, 10, 'f', 3));
}

void monitor::faster()
{
    scene->faster();
}

void monitor::slower()
{
    scene->slower();
}

void monitor::timerEvent(QTimerEvent *e)
{
    //std::cout << "timer" << std::endl;
    Q_UNUSED(e);

    if (!simulationThread){
        simulationThread = new SimulationThread(simulator);
        simulationThread->startThread();
    }

    unsigned int len = scene->logLength();
    if (len && playJustSlider->maximum() != len-1 ){
        playJustSlider->setMaximum(len-1);
    }
    canvas->updateGL();
}

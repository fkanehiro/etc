#include <QGLWidget>

class CanvasWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit CanvasWidget(QWidget *parent = 0 , const char *name = 0);
    ~CanvasWidget();
    virtual void initializeGL();
    virtual void resizeGL( int width, int height );
    virtual void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
signals:
public slots:

private:
    void initViewParameters();
    QPoint lastPos;
    double aspect;
    double pan, tilt, radius;
    double xCenter, yCenter, zCenter;
};

#pragma once

#include <time.h>

class PlaceCalcListener;

//---------------------------------------------------------------------------

class PlaceCalcI
{
public:
    /// Установить listener который будет вызываться по окончании расчёта
    virtual void __stdcall setListener( PlaceCalcListener * listener ) = 0;

    /** \brief Установить шаг сетки
     *  Установка маленького шага улучшает точность определения МП,
     *  но ухудшает быстродействие и наоборот
    */
    virtual void __stdcall setFindStep( unsigned int step_m ) = 0;

    /// Установить время, после которого измерение перестаёт участвовать в расчётах (на 2-ом этапе, после перехода на расчет в потоке)
    virtual void __stdcall setMeasureLifeTime( int seconds ) = 0;

    /// Сбросить все предыдущие измерения
    virtual void __stdcall reset() = 0;

    /** \brief Добавить измерение
     *  \param ownLatitude  Широта собственного носителя
     *  \param ownLongitude Долгота собственного носителя
     *  \param distance_m   Расстояние до объекта, метры 
     *  \param dateTime     Время измерения
    */
    virtual void __stdcall addMeasure( double ownLatitude
                                     , double ownLongitude
                                     , int distance_m
                                     , const time_t & dateTime ) = 0;
    /** \brief Начать расчёт.
     * По окончании расчёта будет вызвана функция PlaceCalcListener::onCalcResult.
    */
    virtual void __stdcall calc() = 0;

protected:
    virtual ~PlaceCalcI() {}
};

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

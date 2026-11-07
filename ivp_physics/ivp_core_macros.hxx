// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef IVP_CORE_MACROS_INCLUDED
#define IVP_CORE_MACROS_INCLUDED

/********************************************************************************
 *	Name:		ivp_core_macros
 *	Description:	inline functions for object core
 ********************************************************************************/

#include <ivp_anomaly_manager.hxx>

// Retail inlines both (e.g. update_cache_object, RVA 0x18A40) with the time
// difference left in ST: the interpolation parameter and each coordinate
// are rounded once, at their double stores.
void IVP_Core::inline_calc_at_quaternion(IVP_Time current_time, IVP_U_Quat *quat) const
{
#if IVP_ENABLE_X87_EMULATION
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    const IVP_U_Float80 d_time = fp.subtract(fp.load(current_time.get_seconds()), fp.load(time_of_last_psi.get_seconds()));
    quat->set_interpolate_smoothly(&q_world_f_core_last_psi, &q_world_f_core_next_psi,
                                   fp.store_double(fp.multiply(d_time, fp.load(i_delta_time))));
#else
    IVP_DOUBLE d_time = current_time - time_of_last_psi;
    quat->set_interpolate_smoothly(&q_world_f_core_last_psi, &q_world_f_core_next_psi, d_time * i_delta_time);
#endif
}

void IVP_Core::inline_calc_at_position(IVP_Time current_time, IVP_U_Point *pos) const
{
#if IVP_ENABLE_X87_EMULATION
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    const IVP_U_Float80 d_time = fp.subtract(fp.load(current_time.get_seconds()), fp.load(time_of_last_psi.get_seconds()));
    for (int axis = 0; axis < 3; ++axis)
        pos->k[axis] = fp.store_double(fp.add(fp.multiply(fp.load(delta_world_f_core_psis.k[axis]), d_time),
                                              fp.load(pos_world_f_core_last_psi.k[axis])));
#else
    IVP_DOUBLE d_time = current_time - time_of_last_psi;
    pos->add_multiple(&pos_world_f_core_last_psi, &delta_world_f_core_psis, d_time);
#endif
}

void IVP_Core::clip_velocity(IVP_U_Float_Point *velocity, IVP_U_Float_Point *rotation_vec)
{
    // perform clipping
    IVP_Anomaly_Limits *al = this->environment->get_anomaly_limits();
    if (this->spin_clipping)
    {
        if (rotation_vec->k[0] > spin_clipping->k[0])
        {
            rotation_vec->k[0] = spin_clipping->k[0];
        }
        else if (rotation_vec->k[0] < -spin_clipping->k[0])
        {
            rotation_vec->k[0] = -spin_clipping->k[0];
        }

        if (rotation_vec->k[1] > spin_clipping->k[1])
        {
            rotation_vec->k[1] = spin_clipping->k[1];
        }
        else if (rotation_vec->k[1] < -spin_clipping->k[1])
        {
            rotation_vec->k[1] = -spin_clipping->k[1]; //@@CB
        }

        if (rotation_vec->k[2] > spin_clipping->k[2])
        {
            rotation_vec->k[2] = spin_clipping->k[2];
        }
        else if (rotation_vec->k[2] < -spin_clipping->k[2])
        {
            rotation_vec->k[2] = -spin_clipping->k[2];
        }
    }
    else
    {
#ifdef IVP_FAST_WHEELS_ENABLED
        if ((!this->car_wheel) || (this->max_surface_deviation))
#endif
        {
            IVP_DOUBLE square_rot_speed = rotation_vec->quad_length();

            IVP_DOUBLE max_rot_speed = al->get_max_angular_velocity_per_psi() * this->environment->get_inv_delta_PSI_time();
            IVP_DOUBLE quad_max_rot_speed = max_rot_speed * max_rot_speed;

            if (square_rot_speed > quad_max_rot_speed)
            {
                IVP_Anomaly_Manager *am = this->environment->get_anomaly_manager();
                am->max_angular_velocity_exceeded(al, this, rotation_vec);
            }
        }
    }

    IVP_DOUBLE max_speed = al->get_max_velocity();
    IVP_DOUBLE quad_max_speed = max_speed * max_speed;
    IVP_DOUBLE square_speed = velocity->quad_length();

    if (square_speed > quad_max_speed)
    {
        IVP_Anomaly_Manager *am = this->environment->get_anomaly_manager();
        am->max_velocity_exceeded(al, this, velocity);
    }
}

#endif // IVP_CORE_MACROS_INCLUDED
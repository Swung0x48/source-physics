// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_HULL_MANAGER_MACROS_INCLUDED
#define IVP_HULL_MANAGER_MACROS_INCLUDED

IVP_BOOL IVP_Hull_Manager::are_events_in_hull()
{
    if (sorted_synapses.find_min_value() - hull_value_next_psi < 0.0f)
    { // no other synapses need to be checked
        return IVP_TRUE;
    }
    else
    {
        return IVP_FALSE;
    }
}

void IVP_Hull_Manager::check_hull_synapses(IVP_Environment *pEnv)
{
    // check all synapses

    // eventually throw hull limit exceeded event
    IVP_FLOAT syn_val;
    // Retail (commit_all_hull_managers 0x1EB2B, 0x1EBBB, 0x1EC41: mov edi, 0x64): at most 100 + 2 events per hull
    // manager and PSI, the rest wait for the next PSI; the anomaly limit (default 1000) came with the Source port.
    (void)pEnv;
    int maxcnt = 100;
    while ((syn_val = sorted_synapses.find_min_value() - hull_value_next_psi) < 0.0f)
    { // no other synapses need to be checked
        // syn hull event is to be fired
        IVP_Listener_Hull *syn = (IVP_Listener_Hull *)sorted_synapses.find_min_elem(); // corresponding synapse
        syn->hull_limit_exceeded_event(this, syn_val);                                 // ATT: syn MUST update/remove itself/its peer from min_hash!
        if (maxcnt-- < 0)
            break;
    }
}

void IVP_Hull_Manager::increase_hull_by_x(IVP_Time t_now, IVP_FLOAT delta_time, IVP_FLOAT gradient_, IVP_FLOAT center_gradient_)
{
#if IVP_ENABLE_X87_EMULATION
    // Retail RVA 0x1E750: dt and the new gradient's product stay in ST; each
    // stored value is rounded once, at its float store.
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    const IVP_U_Float80 dt = fp.subtract(fp.load(t_now.get_seconds()), fp.load(last_vpsi_time.get_seconds()));
    last_vpsi_time = t_now;
    hull_value_last_vpsi = fp.store_float(fp.add(fp.multiply(dt, fp.load(gradient)), fp.load(hull_value_last_vpsi)));
    hull_center_value_last_vpsi = fp.store_float(fp.add(fp.multiply(dt, fp.load(center_gradient)), fp.load(hull_center_value_last_vpsi)));
    center_gradient = center_gradient_;
    const IVP_U_Float80 g = fp.multiply(fp.load(gradient_), fp.load(IVP_HULL_MANAGER_GRADIENT_FACTOR));
    gradient = fp.store_float(g);
    hull_value_next_psi = fp.store_float(fp.add(fp.multiply(g, fp.load(delta_time)), fp.load(hull_value_last_vpsi)));
#else
    IVP_FLOAT dt = t_now - last_vpsi_time;
    hull_value_last_vpsi += gradient * dt;
    hull_center_value_last_vpsi += center_gradient * dt;
    last_vpsi_time = t_now;
    gradient = gradient_ * IVP_HULL_MANAGER_GRADIENT_FACTOR;
    center_gradient = center_gradient_;

    IVP_FLOAT delta = delta_time * gradient;
    hull_value_next_psi = hull_value_last_vpsi + delta;
#endif
    IVP_ASSERT(hull_value_next_psi >= 0.0f);
}

#endif // IVP_HULL_MANAGER_MACROS_INCLUDED
// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#pragma implementation "ivp_cache_object.hxx"
#pragma implementation "ivp_cache_ledge_point.hxx"
#endif

#include <ivp_cache_ledge_point.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_core.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_physics.hxx>
#include <ivu_string.hxx>
#include <ivu_matrix_macros.hxx>

void IVP_Cache_Object_Manager::invalid_cache_object(IVP_Real_Object *object)
{
    if (object->cache_object)
    {
        IVP_ASSERT(object->cache_object->reference_count == 0);
        IVP_ASSERT(object->cache_object->object == object);

        object->cache_object->object = NULL;
        object->cache_object = NULL;
    }
}

IVP_Cache_Object *IVP_Cache_Object_Manager::get_cache_object(IVP_Real_Object *object)
{
    IVP_Cache_Object *co;
    int attempts = 0;
    while ((co = cache_object_at(reuse_loop_index))->reference_count)
    {
        reuse_loop_index++;
        if (reuse_loop_index >= n_cache_objects)
        {
            reuse_loop_index = 0;
        }
        if (++attempts >= n_cache_objects)
        {
            IVP_ASSERT(0 && "All cache objects are referenced - cannot evict");
            ivp_message("IVP_Cache_Object_Manager::get_cache_object: all %d cache objects are referenced\n",
                        n_cache_objects);
            CORE;
            return NULL;
        }
    }
    reuse_loop_index++;
    if (reuse_loop_index >= n_cache_objects)
    {
        reuse_loop_index = 0;
    }
    IVP_Real_Object *obj = co->object;
    if (obj)
    {
        invalid_cache_object(obj); // invalid old caches for object
    }
    co->object = object;
    object->cache_object = co;

    co->valid_until_time_code = 0; // invalid cache for moving objects
    // and set cache for not simulated objects
    if (!IVP_MTIS_SIMULATED(object->flags.object_movement_state))
    {
        co->update_cache_object();
    }

    return co;
}

#ifdef DEBUG
void IVP_Cache_Object_Manager::check()
{
    IVP_ASSERT(reuse_loop_index < n_cache_objects);
    for (int i = 0; i < n_cache_objects; i++)
    {
        IVP_Cache_Object *co = cache_object_at(i);
        IVP_ASSERT(co->reference_count == 0);
        IVP_IF(co->object)
        {
            IVP_ASSERT(co->object->cache_object == co);
        }
    }
}
#endif

IVP_Cache_Object_Manager::IVP_Cache_Object_Manager(int number_of_cache_elements)
{
    IVP_ASSERT(number_of_cache_elements > 0);
    this->n_cache_objects = (number_of_cache_elements > 0) ? number_of_cache_elements : 1;
    unsigned int size = sizeof(IVP_Cache_Object);
    size = (size + 0xf) & ~0xf; // align 16
    this->cache_objects_buffer = (char *)p_calloc(size, this->n_cache_objects);
    this->reuse_loop_index = 0;
}

IVP_Cache_Object_Manager::~IVP_Cache_Object_Manager()
{
    int i;
    for (i = 0; i < this->n_cache_objects; i++)
    {
        IVP_Cache_Object *co = cache_object_at(i);
        if (co->object)
        {
            this->invalid_cache_object(co->object);
        }
    }
    P_FREE(this->cache_objects_buffer);
}

void IVP_Cache_Object::update_cache_object()
{
    IVP_Environment *env = object->get_environment();

    IVP_Time current_time = env->get_current_time();
    valid_until_time_code = env->get_current_time_code();

    IVP_Core *core = object->get_core();
    if (current_time - core->time_of_last_psi == 0.0f)
    {
        q_world_f_object = core->q_world_f_core_last_psi;
        m_world_f_object.vv.set(&core->pos_world_f_core_last_psi);
        core_pos.set(&core->pos_world_f_core_last_psi);
    }
    else
    {
        object->get_core()->inline_calc_at_quaternion(current_time, &q_world_f_object);
        object->get_core()->inline_calc_at_position(current_time, &m_world_f_object.vv);
        core_pos.set(&m_world_f_object.vv);
    }
    q_world_f_object.set_matrix(&m_world_f_object);

    if (!object->flags.shift_core_f_object_is_zero)
    {
        m_world_f_object.vmult4(&object->shift_core_f_object, &m_world_f_object.vv);
    }

    if (object->q_core_f_object)
    {
#if IVP_ENABLE_X87_EMULATION
        // Retail RVA 0x18BED inlines the product with its own summation order;
        // each component is rounded once, at its double store.
        IVP_X87_Arithmetic fp(IVP_X87_Control::current());
        const IVP_U_Quat *a = &q_world_f_object, *b = object->q_core_f_object;
        const IVP_U_Float80 x1 = fp.load(a->x), y1 = fp.load(a->y), z1 = fp.load(a->z), w1 = fp.load(a->w);
        const IVP_U_Float80 x2 = fp.load(b->x), y2 = fp.load(b->y), z2 = fp.load(b->z), w2 = fp.load(b->w);
        const IVP_U_Float80 xx = fp.subtract(fp.add(fp.add(fp.multiply(w1, x2), fp.multiply(y1, z2)), fp.multiply(x1, w2)), fp.multiply(z1, y2));
        const IVP_U_Float80 yy = fp.subtract(fp.add(fp.add(fp.multiply(w1, y2), fp.multiply(z1, x2)), fp.multiply(y1, w2)), fp.multiply(x1, z2));
        const IVP_U_Float80 zz = fp.subtract(fp.add(fp.add(fp.multiply(w1, z2), fp.multiply(z1, w2)), fp.multiply(x1, y2)), fp.multiply(y1, x2));
        const IVP_U_Float80 ww = fp.subtract(fp.subtract(fp.subtract(fp.multiply(w1, w2), fp.multiply(x1, x2)), fp.multiply(y1, y2)), fp.multiply(z1, z2));
        q_world_f_object.x = fp.store_double(xx); q_world_f_object.y = fp.store_double(yy);
        q_world_f_object.z = fp.store_double(zz); q_world_f_object.w = fp.store_double(ww);
#else
        q_world_f_object.inline_set_mult_quat(&q_world_f_object, object->q_core_f_object);
#endif
        q_world_f_object.set_matrix(&m_world_f_object);
    }
}

void IVP_Cache_Object::transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const
{
#if IVP_ENABLE_X87_EMULATION
    // Retail RVA 0x18CA0: the offsets stay extended; each axis sums the rows
    // from the last one and is rounded once, at its double store.
    IVP_X87_Arithmetic fp(IVP_X87_Control::current());
    IVP_U_Float80 d[3];
    for (int i = 0; i < 3; ++i)
        d[i] = fp.subtract(fp.load(P_world->k[i]), fp.load(m_world_f_object.vv.k[i]));
    for (int axis = 0; axis < 3; ++axis)
        P_object_out->k[axis] = fp.store_double(fp.add(fp.add(
            fp.multiply(fp.load(m_world_f_object.get_elem(2, axis)), d[2]),
            fp.multiply(fp.load(m_world_f_object.get_elem(1, axis)), d[1])),
            fp.multiply(fp.load(m_world_f_object.get_elem(0, axis)), d[0])));
#else
    this->m_world_f_object.inline_vimult4(P_world, P_object_out);
#endif
}

void IVP_Cache_Object::transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Float_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult4(P_world, P_object_out);
}

void IVP_Cache_Object::transform_position_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Point *P_world_out) const
{
    // Retail RVA 0x18D10 uses the same X/Y double spills as vmult4 (0xF5F0).
    this->m_world_f_object.vmult4(P_object, P_world_out);
}

void IVP_Cache_Object::transform_vector_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const
{
#if IVP_ENABLE_X87_EMULATION
    this->m_world_f_object.vimult3(P_world, P_object_out);
#else
    this->m_world_f_object.inline_vimult3(P_world, P_object_out);
#endif
}

void IVP_Cache_Object::transform_vector_to_object_coords(const IVP_U_Float_Point *P_world, IVP_U_Float_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult3(P_world, P_object_out);
}

void IVP_Cache_Object::transform_vector_to_world_coords(const IVP_U_Point *P_object, IVP_U_Point *P_world_out) const
{
#if IVP_ENABLE_X87_EMULATION
    this->m_world_f_object.vmult3(P_object, P_world_out);
#else
    this->m_world_f_object.inline_vmult3(P_object, P_world_out);
#endif
}

void IVP_Cache_Object::transform_vector_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Float_Point *P_world_out) const
{
    this->m_world_f_object.inline_vmult3(P_object, P_world_out);
}

============
Type Erasure
============

This is my short guide to **type erasure**, a technique which enables to *abstract* the underlying class of a function, and use them interchangeably.
In this regard, it is similar to (dynamic) **polymorphism**, but enables to have *value objects* and not *pointers*.


**Other resources**

- Article: `C++ 'Type Erasure' Explained <https://davekilian.com/cpp-type-erasure.html>`_;
- Keynote talk by Klaus Iglberger: `There Is No Silver Bullet to Solve All C++ Software Problems <https://youtu.be/m3UmABVf55g?si=DDVd0sagXTbSYstu>`_;


Problem statement
=================

The example focuses on implementing some ``Function`` objects (implemented as classes, for simplicity) that can be then rendered using a generic ``Drawer`` function (e.g. that might render to pyplot, or gnuplot).


Dynamic polymorphism
====================

If we think of dynamic polymorphism, to achieve the desired behavior, first we must define our **interfaces**:

.. literalinclude:: src/dynamic_polymorphism.cpp
  :language: cpp
  :lines: 7-16
  
To simplify, ``Function`` is a functor interface, and each concrete type must implement an ``operator()`` that takes a ``double`` and returns a ``double``, and a ``describe`` function that yields some sort of description of the function itself.

The ``Drawer`` interface, instead, simply requires that the concrete type to have a ``draw`` method which accept a ``Function*``, and the range in which to draw the function; note that here we must use a pointer to the interface, otherwise dynamic polymorphism wouldn't work.
  
Then, we implement **concrete types** for the function interface; here we create a wrapper around ``std::sqrt`` and a parametric parabola function :math:`ax^2 + bx + c`:

.. literalinclude:: src/dynamic_polymorphism.cpp
  :language: cpp
  :lines: 19-31

We also implement a *useless* drawer, for sake of demonstration

.. literalinclude:: src/dynamic_polymorphism.cpp
  :language: cpp
  :lines: 33-39

Once we defined the interfaces and implemented some concrete types, we can start creating some user-defined functions; here we create some type aliases, and proceed implementing a function that draws all functions in a given collection:

.. literalinclude:: src/dynamic_polymorphism.cpp
  :language: cpp
  :lines: 42-47

Finally we can create our ``main`` application that creates a polymorphic vector of functions, and use the ``VoidDrawer`` to display them:

.. literalinclude:: src/dynamic_polymorphism.cpp
  :language: cpp
  :lines: 49-56


Static polymorphism
===================

We implement the same problem now using static polymorphism, i.e. by using **templates**.

By using templates, we can omit the common interface, as the constraint will be enforced at compile time. In this way, we can simply define the concrete type of the function with no inheritance:

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 10-22

The drawer is a little bit more complicated, as the function call ``draw`` must be templated for any ``Function`` interface.
Here, if we pass a ``Function`` object that doesn't comply with the interface (having ``operator()(double&) -> double`` and ``describe() -> string``) will result in a compilation error:

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 24-33

**Note:** to make the interface requirement more explicit, we could have used C++20 `concepts <https://en.cppreference.com/w/cpp/language/constraints>`_, but that would have make the code more complex.


Now, following the dynamic polymorphism examplew, we need to implement a ``draw_function`` like the following:

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 37-40

When using templates, such definition does not work; indeed, we cannot clearly define a `FunctionCollection` type as in the other case: since there's no common base class, we cannot rely on *pointers to base class* to store heterogeneous types.

To tackle this issue, we must use `variants <https://en.cppreference.com/w/cpp/utility/variant>`_, i.e. a type-safe union, to store the 2 concrete types of function into a single object. 
Doing so, we can create a collection simply as a ``std::vector`` of values (and no more of pointers).

Now we could be tempted to do the following:

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 43-45, 37-40

This is still **incorrect** (such code won't compile), since ``f`` in the for-loop is a variant object, not the concrete function type we want. 
To call the proper ``draw`` function of the drawer based on the actual function stored in the variant, we must use a `visitor <https://en.cppreference.com/w/cpp/utility/variant/visit2>`_:

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 43-50

Finally, we can define the ``main`` function; in this case we don't rely anymore on pointers, but rather on value objects.

.. literalinclude:: src/template_polymorphism.cpp
  :language: cpp
  :lines: 52-59

Bonus: concept version
----------------------
In C++20, we might use concepts to better define templated interfaces.

Indeed, in the templated version of ``VoidDrawer``, any object that has a ``describe()`` is technically valid (as long as it is formattable by the ``fmt`` library used in the example).
For instance, of ``SqrtFunction::describe`` returned a ``double``, the code would have compiled just fine.

In this version we create the ``function_concept`` concept that ensures that, given a type ``T`` and a ``const double&`` value ``value``: 

- ``obj(value)`` is a valid expression;
- the outcome of the expression ``{ obj(value) }`` is convertible to a ``double``;
- ``obj.describe()`` is a valid expression;
- the outcome ``{ obj.describe() }`` can be converted to a ``std::string``.

.. literalinclude:: src/concept_polymorphism.cpp
  :language: cpp
  :lines: 27-40

Value based polymorphism - Type erasure
=======================================

As the name suggests, **type erasure** is a technique that aims at *hiding* the underlying type we want to use.

Let us see how to create a type-erased function; first, we need to specify the **interface** for a ``Function``, i.e.:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 53-56

This is exactly the interface we had for the dynamic polymorphism implementation (except we changed the name of such purely virtual interface).

Now, we define a **concrete** function object as a templated class ``FunctionModel`` which **owns** the actual function implementation:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 58-66

Here we observe that:

- ``FunctionModel`` inherits from ``FunctionConcept``, thus it includes some polymorphic behavior;
- ``FunctionModel`` is a templated class, and requires an actual ``CallableFunction`` to become a **concrete** type. 
  We observe here that the arbitrary ``CallableFunction`` might not inherit from ``FunctionConcept``; it's the wrapper provided by the ``FunctionModel`` that will encapsulate the polymorphic behavior. Of course, if ``CallableFunction`` doesn't conform with the interface, a compilation error will be thrown.

At this point, we have all the *ingredients* to create a **type-erased** ``Function`` object:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 24-50

Here, ``FunctionConcept`` and ``FunctionModel`` are private members of ``Function``, and thus cannot be accessed outside its definition; doing so, we prevent concrete types to inherit from ``Function::FunctionConcept``.

The ``Function`` object *stores the implementation* in a (unique) pointer to ``FunctionConcept``, in order to effectively enable the polymorphic behavior.
Such pointer is instantiated using a templated constructor that assigns ``_fun`` a pointer to ``FunctionModel<CallableFunction>``, i.e., it creates a concrete ``FunctionModel`` type based on the provided input.

To have a proper value-based semantic of the interface, ``Function`` must re-export the *interfaced functions*, but without the need of the virtual overload.


Program overview
----------------

Based on this explanation of type erasure, we achieve a different program.

We can start off defining the inheritance-free concrete types for the function (as in the static polymorphism example):

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 10-22

Using type-erasure, we *can* define the interface after the definition of the concrete types (tthat wasn't possible using standard dynamic polymorphism), as required by the drawing facility;
in particular, also the ``Drawer`` interface can be type-erased:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 68-70, 73-98

Based on these interfaces, we can define the template-less `draw_functions` that accept values as inputs, as well as a drawer:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 100-112

Finally, the ``main`` application consists in a ``std::vector`` of ``Function`` value objects, and not pointer, and a call to ``draw_functions`` with the provided ``VoidDrawer``:

.. literalinclude:: src/value_based_polymorphism.cpp
  :language: cpp
  :lines: 114-120

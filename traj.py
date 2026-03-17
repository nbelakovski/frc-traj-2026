import marimo

__generated_with = "0.20.4"
app = marimo.App(width="medium")


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    # Trajectory modeling for FRC Rebuilt 2026

    - Equations of motion for ball in a vacuum
        - Analytical solution
        - Numerical solution
        - Comparison of analytical vs numerical
    - Equations of motion for ball in an atmosphere
        - Numerical solution
        - Comparison vs vacuum
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Equations of Motions (EOMs) for a ball in a vacuum

    ### Drawing and definitions

    ```
    y
    ^

    |
    |     .
    |    /
    |   /
    |  /
    |_/_______  >x
    ```

    - $x$ - distance traveled by the ball in the horizontal direction
    - $\dot{x}$ - velocity of the ball in the horizontal direction (pronounced x dot)
    - $\ddot{x}$ - acceleration of the ball in the horizontal direction (x double dot)
    - $y, \dot{y}, \ddot{y}$ - distance, velocity, and acceleartion of the ball in the vertical direction
    - Equations of Motion (EOMs) - a set of differential equations desribing the translational and rotational acceleration of an object (in our case x and y accelerations)

    ### Equations of motion for a ball in a vacuum

    We will not go through how to derive equations of motion in detail, but we will give a brief explanation


    $\ddot{x} = 0 \newline \ddot{y} = -g$
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ### Next steps: analytical and/or numerical solution

    #### Analytical solution

    Integrate $\ddot{x}$ with respect to time to get $\dot{x}$

    $\dot{x}(t) = c_1$

    Integrate again to get $x$

    $x(t) = c_1 t + c_2$

    Similarly, integrating $y$ gives

    $\dot{y}(t) = -gt + c_3 \newline
    y(t) = -\frac{1}{2}gt^2 + c_3 t + c_4$

    Anayltical solutions are very easy to work with
    """)
    return


@app.cell(hide_code=True)
def _():
    # Imports and definitions
    import marimo as mo
    import plotly.graph_objects as go
    import numpy as np
    from scipy.integrate import solve_ivp
    # Program constants
    g_mps2 = 9.81
    m_kg = 0.227
    S_m2 = np.pi * (0.15/2)**2  # cross sectional area of a 6" ~15cm diameter ball
    rho_kgpm3 = 1.225
    cd = 0.5  # https://scienceworld.wolfram.com/physics/DragCoefficient.html
    robot_length_m = 0.761
    hub_center_m = 182.11 * 0.0254
    y0_m = 0.2  # initial y position
    hub_height_m = 1.83  # 72 inches
    v0_mps_slider = mo.ui.slider(value=8, start=1, stop=10, step=0.1, show_value=True)  # initial speed (m/s)
    theta0_rad_slider = mo.ui.slider(value=79, start=45, stop=90, step=0.1, show_value=True)  # initial launch angle (radians)
    dist_m_slider = mo.ui.slider(value=2, start=0, stop=5, step=0.1, show_value=True)  # distance to hub (meters)
    mo.hstack([
        mo.vstack([
            mo.md("Initial speed (m/s)"),
            v0_mps_slider
        ]),
        mo.vstack([
            mo.md("Initial launch angle (degrees)"),
            theta0_rad_slider
        ]),
        mo.vstack([
            mo.md("Turret center to hub center (meters)"),
            dist_m_slider
        ])
    ]
    )
    return (
        S_m2,
        cd,
        dist_m_slider,
        g_mps2,
        go,
        hub_center_m,
        hub_height_m,
        m_kg,
        mo,
        np,
        rho_kgpm3,
        robot_length_m,
        solve_ivp,
        theta0_rad_slider,
        v0_mps_slider,
        y0_m,
    )


@app.cell(hide_code=True)
def _(go, np, robot_length_m):
    def field_figure(robot_pos = 1.5):
        # Dimensions from https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dwg-complete.pdf
        hub_center = 182.11
        hub_width = 47
        hub_target_width = 41.73  # no bigger than a womprat
        hub_diffuser_inset = (47-38.07)/2  # assuming symmetrical 
        hub_base_height = 49.75
        hub_diffuser_height_left = 60  # estimated
        hub_diffuser_height_right = 55  # estimated
        hub_slope = (hub_diffuser_height_left - hub_diffuser_height_right) / (hub_width - 2*hub_diffuser_inset)
        hub_target_base_inset_left = hub_diffuser_inset + 1.7
        hub_target_base_inset_right = hub_diffuser_inset + 5.38
    
        hub = [
            (hub_center - hub_width/2, 0),
            (hub_center - hub_width/2, hub_base_height),
            (hub_center - hub_width/2 + hub_diffuser_inset, hub_diffuser_height_left),
    
            (hub_center - hub_width/2 + hub_target_base_inset_left, hub_diffuser_height_left - (hub_target_base_inset_left - hub_diffuser_inset)*hub_slope),
        
            (hub_center - hub_target_width/2, 72),
            (hub_center, 72),
            (hub_center + hub_target_width/2, 72),
        
            (hub_center + hub_width/2 - hub_target_base_inset_right, hub_diffuser_height_left - (hub_width - hub_diffuser_inset - hub_target_base_inset_right)*hub_slope),
        
            (hub_center + hub_width/2 - hub_diffuser_inset, 55),
            (hub_center + hub_width/2, 49.75),
            (hub_center + hub_width/2, 0),
        ]
    
        tower = [
            (0, 0),
            (0, 35.125 + 1.75),
            (40, 35.125 + 1.75),
            (40, 72.125),
            (43.51, 72.125),
            (43.51, 0),
            (40, 0),
            (40, 35.125),
            (0, 35.125),
        ]
    
        in2m = 0.0254

        robot = np.array([
            (0, 0.1),
            (0, 0.1+3*in2m),
            (robot_length_m, 0.1+3*in2m),
            (robot_length_m, 0.1),
            (0, 0.1)
        ])
    
        robot[:, 0] += robot_pos

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=[point[0]*in2m for point in hub], y=[point[1]*in2m for point in hub], mode='lines', line={'color': 'rgba(200, 0, 0, 0.3)'}, showlegend=False))
        fig.add_trace(go.Scatter(x=[point[0]*in2m for point in tower], y=[point[1]*in2m for point in tower], mode='lines', line={'color': 'rgba(200, 0, 0, 0.3)'}, showlegend=False))
        fig.add_trace(go.Scatter(x=robot[:, 0], y=robot[:, 1], mode='lines', line={'color': 'red'}, showlegend=False))
        fig.update_xaxes(range=[-0.5, 6])
        fig.update_yaxes(range=[-0.2, 4])
        return fig

    

    return (field_figure,)


@app.cell(hide_code=True)
def _(dist_m_slider, hub_center_m, np, theta0_rad_slider, v0_mps_slider):
    # Extract values from sliders
    v0_mps = v0_mps_slider.value
    theta0_rad = np.radians(theta0_rad_slider.value)
    x0_dot_mps = v0_mps * np.cos(theta0_rad)  # initial x velocity
    y0_dot_mps = v0_mps * np.sin(theta0_rad)  # initial y velocity
    x0_m = hub_center_m - dist_m_slider.value  # initial x position based on distance to hub
    robot0_m = x0_m - 0.1  # position robot so that ball starts 10 cm in front of it
    return robot0_m, x0_dot_mps, x0_m, y0_dot_mps


@app.cell(hide_code=True)
def _(
    field_figure,
    g_mps2,
    go,
    hub_height_m,
    np,
    robot0_m,
    x0_dot_mps,
    x0_m,
    y0_dot_mps,
    y0_m,
):
    # With an analytical solution, we can calculate exactly when the ball will reach the hub height of 72 inches (1.83 meters)
    a = -1/2*g_mps2
    b = y0_dot_mps
    c = y0_m - hub_height_m
    tf_s = (-b - np.sqrt(b**2 - 4*a*c))/(2*a)

    t_s = np.linspace(0, tf_s, 10000)
    x_m = lambda t_s: x0_m + x0_dot_mps * t_s
    y_m = lambda t_s: y0_m + y0_dot_mps * t_s - 1/2 * g_mps2 * t_s**2


    analytical_figure = field_figure(robot_pos=robot0_m)
    analytical_figure.add_trace(go.Scatter(x=x_m(t_s), y=y_m(t_s), mode='lines', name='Analytical Solution'))
    analytical_figure.update_layout(title='Analytical solution for trajectory in a vacuum', xaxis_title='x (m)', yaxis_title='y (m)', legend_title='Legend')
    analytical_figure.show()
    return t_s, tf_s, x_m, y_m


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    #### Numerical solution

    Sometimes analytical solution is not available, but we still have the differential equations, and we know where we start.

    If we assume the acceleration is constant over a very small slice of time, we can propagate our state and re-evalaute the differential equations.

    In this case, there's nothing to re-evaluate since our DE's are constant for all time, not just a time slice, but when learning about numerical solutions it's useful to experiment on DEs with an analytical solution so you can compare the methods.

    #### The simplest way to do numerical integration: Euler's method


    $x_1 = x_0 + \dot{x}_0\Delta t \newline
    x_2 = x_1 + \dot{x}_1\Delta t \newline
    ...$
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## EOMs for trajectory in an atmosphere

    ```
    y
    ^
    |

    |     .
    |    /
    |   /
    |  /  θ
    |_/_______  --> x
    ```


    $\hat{v} = \cos(\theta) \hat{x} + \sin(\theta)\hat{y}$

    $\hat{D} = -\hat{v}$

    $D = c_d q S$

    where
    - $c_d$ is the drag coefficient
    - $q$ is the dynamic pressure ($\frac{1}{2} \rho v^2$ where $\rho$ is air density)
    - $S$ is cross sectional area of the ball



    $\ddot{x} = -\frac{1}{m} c_d q S \cos(\theta)$

    $\ddot{y} = -\frac{1}{m} c_d q S \sin(\theta) - g$

    We can simplify a little bit, since $v^2\cos \theta = v v_x$
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    $\dot{X} = f(t,X)$

    $X = \begin{bmatrix}
               x \\
               y \\
               \dot{x} \\
               \dot{y}
             \end{bmatrix}$

    $\dot{X} = \begin{bmatrix}
               \dot{x} \\
               \dot{y} \\
               \ddot{x} \\
               \ddot{y}
             \end{bmatrix} = \begin{bmatrix}
               \dot{x} \\
               \dot{y} \\
               -\frac{1}{m} c_d q S \cos(\theta) \\
               -\frac{1}{m} c_d q S \sin(\theta) - g
             \end{bmatrix} = \begin{bmatrix}
               \dot{x} \\
               \dot{y} \\
               -\frac{1}{m} c_d \frac{1}{2} \rho S v v_x \\
               -\frac{1}{m} c_d \frac{1}{2} \rho S v v_y - g
             \end{bmatrix}$
    """)
    return


@app.cell(hide_code=True)
def _(
    S_m2,
    cd,
    g_mps2,
    hub_height_m,
    m_kg,
    np,
    rho_kgpm3,
    solve_ivp,
    x0_dot_mps,
    x0_m,
    y0_dot_mps,
    y0_m,
):
    def eoms(t, x):
        xdot = np.zeros_like(x)
        vx_mps = x[2]
        vy_mps = x[3]
        xdot[0] = vx_mps
        xdot[1] = vy_mps
        v_mps = np.sqrt(vx_mps**2 + vy_mps**2)
        q = 0.5 * rho_kgpm3
        xdot[2] = - (1/m_kg) * cd * q * S_m2 * v_mps * vx_mps
        xdot[3] = - (1/m_kg) * cd * q * S_m2 * v_mps * vy_mps - g_mps2
        return xdot

    # Initial conditions
    initial_conditions = [x0_m, y0_m, x0_dot_mps, y0_dot_mps]
    # Time span for simulation
    t_span_s = (0, 3)  # simulate for a couple seconds
    t_eval_s = np.linspace(t_span_s[0], t_span_s[1], 1000)
    # Solve the ODE
    hits_ground = lambda t_s, X: X[1] - hub_height_m   # event function to stop integration when y=0 (hits the ground)
    # hits_ground.terminal = True
    hits_ground.direction = -1  # To trigger the logic only when hitting the ground and now when launching
    solution = solve_ivp(eoms, t_span_s, initial_conditions, t_eval=t_eval_s, events=hits_ground)

    # Euler forward with fixed time step
    euler = [np.array(initial_conditions)]
    dt_s = 0.01
    for _ in np.arange(0, solution.t[-1] + dt_s, dt_s):
        euler.append(euler[-1] + eoms(0, euler[-1])*dt_s)
    return (solution,)


@app.cell(hide_code=True)
def _(field_figure, go, hub_height_m, np, robot0_m, solution, t_s, x_m, y_m):

    # make a plotly plot
    fig = field_figure(robot_pos=robot0_m)

    # Compare to vacuum trajectory
    fig.add_trace(go.Scatter(x=x_m(t_s), y=y_m(t_s), mode='lines', name='Vacuum Trajectory'))

    # Add drag trajectory
    idx_past_hub_height = np.where(solution.y[1] > hub_height_m)[0][-1] + 1
    fig.add_trace(go.Scatter(x=solution.y[0][:idx_past_hub_height], y=solution.y[1][:idx_past_hub_height], mode='lines', name='Drag Trajectory'))

    # Add euler trajectory (uncomment when it's time to discuss how expensive euler is)
    # fig.add_trace(go.Scatter(x=[state[0] for state in euler], y=[state[1] for state in euler], mode='lines', name=f'Euler Trajectory ({len(euler)} points)'))

    # Make the axes of the plot equal
    fig.update_layout(title='Trajectory of a ball with drag vs vacuum', xaxis_title='x (m)', yaxis_title='y (m)', legend_title='Legend')
    fig.show()
    return (idx_past_hub_height,)


@app.cell
def _(
    g_mps2,
    go,
    idx_past_hub_height,
    np,
    solution,
    tf_s,
    x_m,
    y0_dot_mps,
    y_m,
):
    figerr = go.Figure()
    # plot error between numerical solution with drag and vacuum solution
    # I need to plot x error for a given y height, so I need to normalize the time steps by time to apex for each solution.
    time_to_apex_vacuum_s = y0_dot_mps / g_mps2
    time_to_apex_drag_s = solution.t[np.where(solution.y[3] < 0)[0][0]]  # time when y velocity becomes negative

    error_x = [0]
    error_y = [0]
    t_error_s = np.linspace(0, 2, 1000)
    for t_error_si in t_error_s:
        t_vacuum = t_error_si * time_to_apex_vacuum_s
        t_drag = t_error_si * time_to_apex_drag_s
        x_vacuum = x_m(t_vacuum)
        y_vacuum = y_m(t_vacuum)
        x_drag = np.interp(t_drag, solution.t, solution.y[0])
        y_drag = np.interp(t_drag, solution.t, solution.y[1])
        error_x.append(x_drag - x_vacuum)
        error_y.append(y_drag - y_vacuum)


    # find the time when vacuum solution enter hub_height
    figerr.add_vline(x=tf_s/time_to_apex_vacuum_s, line_dash='dash', line_color='purple', annotation_text='Vacuum enters hub', annotation_position='top right')
    figerr.add_vline(x=solution.t[idx_past_hub_height]/time_to_apex_drag_s, line_dash='dash', line_color='orange', annotation_text='Drag enters hub', annotation_position='top left')
    figerr.add_trace(go.Scatter(x=t_error_s, y=error_x, mode='lines', name='x error'))
    figerr.add_trace(go.Scatter(x=t_error_s, y=error_y, mode='lines', name='y error'))
    sum_squared_errors_x = np.sum([error_xi**2 for error_xi in error_x])
    sum_squared_errors_y = np.sum([error_yi**2 for error_yi in error_y])
    figerr.update_layout(title=f'Error between numerical solution with drag and vacuum solution: SSEx: {sum_squared_errors_x:.2f}, SSEy: {sum_squared_errors_y:.2f}', xaxis_title='Time (s)', yaxis_title='Error (m)', legend_title='Legend')
    figerr.show()
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## TODO: How might we add a drag term to the analytical solution?
    """)
    return


@app.cell
def _(g_mps2, go, solution, x0_dot_mps, y0_dot_mps):
    figvel = go.Figure()
    # plot x and y velocity of numerical solution with drag
    figvel.add_trace(go.Scatter(x=solution.t, y=solution.y[2] - x0_dot_mps, mode='lines', name='x velocity with drag'))
    figvel.add_trace(go.Scatter(x=solution.t, y=solution.y[3] + g_mps2*solution.t - y0_dot_mps, mode='lines', name='y velocity with drag'))
    figvel.show()
    print((solution.y[2][-1] - solution.y[2][0])/(solution.t[-1] - solution.t[0]))
    print((solution.y[3][solution.t.size//2] + g_mps2*solution.t[solution.t.size//2] - solution.y[3][0])/(solution.t[solution.t.size//2] - solution.t[0]))
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    Suppose we fudged the drag term and just made it depend on $\dot{x}$ instead of $v^2$?

    $\dot{X} = \begin{bmatrix}
               \dot{x} \\
               \dot{y} \\
               \ddot{x} \\
               \ddot{y}
             \end{bmatrix} = \begin{bmatrix}
               \dot{x} \\
               \dot{y} \\
               -\frac{1}{m} c_d \frac{1}{2} \rho S v_x \\
               -\frac{1}{m} c_d \frac{1}{2} \rho S v_y - g
             \end{bmatrix}$

    We've simply "dropped" the $v$ term. Now our equation is linear and we can solve it analytically.

    Let's define $k = -\frac{1}{m} c_d \frac{1}{2} \rho S$ to make the notation simpler

    $\ddot{x} = k \dot{x}$

    $\dot{x}(t) = c_1 e^{kt}$

    $x(t) = \frac{c_1}{k} e^{kt} + c_2$

    And for $y$

    $\ddot{y} = k \dot{y} - g$

    $\dot{y}(t) = c_3 e^{kt} + \frac{g}{k}$

    $y(t) = \frac{c_3}{k} e^{kt} + \frac{g}{k}t + c_4$
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    linear_drag_correction_factor_slider = mo.ui.slider(value=1, start=0, stop=10, step=0.1, show_value=True)
    mo.vstack([
        mo.md("Linear Drag Correction Factor"),
        linear_drag_correction_factor_slider
    ])
    return (linear_drag_correction_factor_slider,)


@app.cell
def _(
    S_m2,
    cd,
    field_figure,
    g_mps2,
    go,
    hub_height_m,
    idx_past_hub_height,
    linear_drag_correction_factor_slider,
    m_kg,
    np,
    rho_kgpm3,
    robot0_m,
    solution,
    t_s,
    x0_dot_mps,
    x0_m,
    x_m,
    y0_dot_mps,
    y0_m,
    y_m,
):
    fignew = field_figure(robot_pos=robot0_m)
    k_1pm = - 1 / m_kg * cd * 0.5 * rho_kgpm3 * S_m2 * linear_drag_correction_factor_slider.value  # The last term is just a fudge factor I arrived at by playing with it to make the linear drag solution match the numerical drag solution as closely as possible. It seems to work really well over the range of initial conditions that we care about.
    xlineardrag = x0_dot_mps/k_1pm * np.exp(k_1pm*t_s) + x0_m - x0_dot_mps/k_1pm
    ylineardrag = (y0_dot_mps - g_mps2/k_1pm)/k_1pm * np.exp(k_1pm*t_s) + g_mps2/k_1pm*t_s + y0_m - (y0_dot_mps - g_mps2/k_1pm)/k_1pm
    fignew.add_trace(go.Scatter(x=x_m(t_s), y=y_m(t_s), mode='lines', name='Vacuum Trajectory'))
    fignew.add_trace(go.Scatter(x=solution.y[0][:idx_past_hub_height], y=solution.y[1][:idx_past_hub_height], mode='lines', name='Numerical Drag Trajectory'))
    # Remove some points at the end so the graph looks nicer
    second_index_of_hub_height = np.where(ylineardrag > hub_height_m)[0][-1]
    fignew.add_trace(go.Scatter(x=xlineardrag[:second_index_of_hub_height], y=ylineardrag[:second_index_of_hub_height], mode='lines', name='Linear Drag Trajectory'))
    fignew.show()
    return


@app.cell
def _():
    return


if __name__ == "__main__":
    app.run()

#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        // add code here
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        // add code here
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("/plan_path", std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));
        
        // Publisher for path
        // add code here
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        // add code here
        while (!map_client_->wait_for_service(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for map server to be available...");
        }

        // Request map
        // add code here
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here
    auto response = future.get();
    map_ = response->map;

    RCLCPP_INFO(this->get_logger(), "Mapa úspěšně načtena! Rozměry: %d x %d (Rozlišení: %f)", map_.info.width, map_.info.height, map_.info.resolution);
    
    dilateMap();
    // ********
    // * Help *
    // ********
    /*
    auto response = future.get();
    if (response) {
        ...
    }
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // add code here
    RCLCPP_INFO(this->get_logger(), "Přijat požadavek na plánování trasy!");

    path_.poses.clear();

    // Kontrola, zda už server načetl mapu
    if (map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Nelze plánovat, mapa ještě nebyla načtena!");
        return;
    }

    // Vytáhnutí startu a cíle z požadavku
    auto start_pose = request->start;
    auto goal_pose = request->goal;

    // Zavolání algoritmu A* (později doplníme návratovou hodnotu)
    aStar(start_pose, goal_pose);

    // ********
    // * Help *
    // ********
    /*
    aStar(request->start, request->goal);
    */
    smoothPath();

    response->plan = path_;

    path_pub_->publish(path_);
}

void PlanningNode::dilateMap() {
    // add code here
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    
    int width = map_.info.width;
    int height = map_.info.height;
    
    // Zvolený bezpečný odstup (např. 4 buňky na každou stranu)
    int radius = 10; 

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            
            int idx = y * width + x;
            
            // Pokud je na tomto políčku v původní mapě překážka (hodnota > 50)
            if (map_.data[idx] > 50) {
                
                // Projdeme okolí v zadaném poloměru
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        // Zkontrolujeme, zda nejsme mimo okraje mapy
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            // Chceme to nafouknout do kruhu, ne do čtverce (Pythagorova věta: a^2 + b^2 <= c^2)
                            //if (dx*dx + dy*dy <= radius*radius) {
                                dilatedMap.data[ny * width + nx] = 100; // Nastavíme jako překážku
                            //}
                        }
                    }
                }
            }
        }
    }
    
    // Uložíme upravenou mapu zpět
    map_ = dilatedMap;
    RCLCPP_INFO(this->get_logger(), "Mapa úspěšně dilatována (nafouknuta o %d buněk).", radius);

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here
    // Získání informací o mapě
    double res = map_.info.resolution;
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;

    // Převod startu na indexy v mřížce
    int start_c = (start.pose.position.x - origin_x) / res;
    int start_r = (start.pose.position.y - origin_y) / res;

    // Převod cíle na indexy v mřížce
    int goal_c = (goal.pose.position.x - origin_x) / res;
    int goal_r = (goal.pose.position.y - origin_y) / res;

    RCLCPP_INFO(this->get_logger(), "Plánuji z [%d, %d] do [%d, %d]", start_c, start_r, goal_c, goal_r);

    // Ošetření, aby start a cíl nebyly úplně mimo mapu (ochrana proti pádu)
    int width = map_.info.width;
    int height = map_.info.height;
    if (start_c < 0 || start_r < 0 || start_c >= width || start_r >= height ||
        goal_c < 0 || goal_r < 0 || goal_c >= width || goal_r >= height) {
        RCLCPP_ERROR(this->get_logger(), "Start nebo cíl je mimo mapu!");
        return;
    }

    // 1. Vytvoření seznamů podle nápovědy ze zadání
    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(height * width, false); // Vše je na začátku neprozkoumané

    // 2. Vytvoření startovní a cílové buňky
    Cell cStart(start_c, start_r);
    Cell cGoal(goal_c, goal_r);

    // 3. Vložení startu do openListu
    openList.push_back(std::make_shared<Cell>(cStart));

    // Hlavní vyhledávací smyčka
    while(!openList.empty() && rclcpp::ok()) {
        
        // 1. Najdi buňku v openListu s nejmenším ohodnocením 'f'
        auto current_it = openList.begin();
        std::shared_ptr<Cell> current = *current_it;
        
        for (auto it = openList.begin(); it != openList.end(); ++it) {
            if ((*it)->f < current->f) {
                current = *it;
                current_it = it;
            }
        }

        // 2. Zkontroluj, jestli už nejsme v cíli
        if (current->x == goal_c && current->y == goal_r) {
            
            std::vector<geometry_msgs::msg::PoseStamped> path_poses;
            std::shared_ptr<Cell> curr = current;
            
            // Procházíme přes 'parent' odkaz zpět až na start
            while (curr != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                // Převod indexů zpět na metry!
                pose.pose.position.x = (curr->x * res) + origin_x;
                pose.pose.position.y = (curr->y * res) + origin_y;
                path_poses.push_back(pose);
                
                curr = curr->parent;
            }
            
            // Rekonstrukce jde od cíle ke startu, takže ji musíme otočit (tip ze zadání)
            std::reverse(path_poses.begin(), path_poses.end()); 
            
            // Uložení do naší globální proměnné
            path_.poses = path_poses;
            path_.header.frame_id = "map";
            path_.header.stamp = this->get_clock()->now();
            
            RCLCPP_INFO(this->get_logger(), "Cesta úspěšně nalezena! Počet bodů: %zu", path_poses.size());
            return; // Konec vyhledávání!
        }

        // 3. Přesuneme buňku z openListu do closedListu (už ji prozkoumáváme)
        openList.erase(current_it);
        // Mapování (x, y) na 1D pole podle tipu ze zadání
        closedList[current->y * width + current->x] = true;
        
        // 4. Prozkoumání 8 sousedů (nahoru, dolů, doleva, doprava a úhlopříčky)
        std::vector<std::pair<int, int>> directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}, // Rovně
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonálně
        };

        for (auto dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            // Kontrola, jestli nejsme mimo mapu
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            // Kontrola, jestli už soused není v closedListu
            if (closedList[ny * width + nx]) continue;

            // Kontrola, jestli na políčku není překážka
            // ROS mapy mají hodnoty 0 (volno) až 100 (zeď). -1 je neznámo.
            int map_idx = ny * width + nx;
            int cell_cost = map_.data[map_idx];
            if (cell_cost > 50 || cell_cost < 0) continue; // Považujeme za zeď cokoliv nad 50

            // Výpočet cen
            // hypotenusa (přepona) = 1.0 pro rovný směr, cca 1.41 pro úhlopříčku
            float step_cost = std::hypot(dir.first, dir.second); 
            float tentative_g = current->g + step_cost;
            float h = std::hypot(goal_c - nx, goal_r - ny); // Euklidovská vzdálenost k cíli vzdušnou čarou
            float f = tentative_g + h;

            // Zjistíme, jestli už soused není v openListu
            bool in_open = false;
            for (auto &cell : openList) {
                if (cell->x == nx && cell->y == ny) {
                    in_open = true;
                    // Pokud jsme našli lepší (kratší) cestu k této buňce, aktualizujeme ji
                    if (tentative_g < cell->g) {
                        cell->g = tentative_g;
                        cell->f = f;
                        cell->parent = current;
                    }
                    break;
                }
            }

            // Pokud soused ještě nebyl objeven, přidáme ho do openListu
            if (!in_open) {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = tentative_g;
                neighbor->h = h;
                neighbor->f = f;
                neighbor->parent = current; // Uložíme si, odkud jsme přišli
                openList.push_back(neighbor);
            }
        }
    }

    // Pokud smyčka skončí a my nenašli cíl (např. je obehnán zdí)
    RCLCPP_ERROR(get_logger(), "Unable to plan path.");

    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */
}

void PlanningNode::smoothPath() {
    // add code here
    // Pokud má trasa jen 2 body (start a cíl), nedá se nic vyhlazovat
    if (path_.poses.size() < 3) return;

    // Kopie trasy (podle nápovědy)
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    
    // Parametry pro gradient descent (můžeš s nimi pak experimentovat)
    float weight_data = 0.1;   // Jak moc se chceme držet původní A* trasy (aby to nenarazilo do zdi)
    float weight_smooth = 0.5; // Jak moc chceme body vyhlazovat mezi sebou
    float tolerance = 0.00001; // Jaká změna nám už stačí k ukončení
    int max_iterations = 1000; // Maximální počet iterací (ochrana proti nekonečné smyčce podle tipu)

    float change = tolerance;
    int iterations = 0;

    // Smyčka běží, dokud se trasa mění a dokud jsme nepřekročili limit iterací
    while (change >= tolerance && iterations < max_iterations) {
        change = 0.0;
        
        // Procházíme všechny body kromě startu (0) a cíle (size - 1), ty musí zůstat na místě!
        for (size_t i = 1; i < path_.poses.size() - 1; ++i) {
            
            // X souřadnice
            float aux_x = newPath[i].pose.position.x;
            newPath[i].pose.position.x += weight_data * (path_.poses[i].pose.position.x - newPath[i].pose.position.x) +
                                          weight_smooth * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x - 2.0 * newPath[i].pose.position.x);
            change += std::abs(aux_x - newPath[i].pose.position.x);

            // Y souřadnice
            float aux_y = newPath[i].pose.position.y;
            newPath[i].pose.position.y += weight_data * (path_.poses[i].pose.position.y - newPath[i].pose.position.y) +
                                          weight_smooth * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y - 2.0 * newPath[i].pose.position.y);
            change += std::abs(aux_y - newPath[i].pose.position.y);
        }
        iterations++;
    }

    // Uložíme vyhlazenou trasu zpět
    path_.poses = newPath;
    RCLCPP_INFO(this->get_logger(), "Trasa vyhlazena v %d iteracích.", iterations);

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
    x = c;
    y = r;
    f = 0.0f;
    g = 0.0f;
    h = 0.0f;
    parent = nullptr;
}

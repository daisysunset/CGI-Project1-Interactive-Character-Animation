using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class keyboard_control : MonoBehaviour
{
    private string bvh_fname1 = "Assets//BVH//walk.bvh";
    private string bvh_fname2= "Assets//BVH//cartwheel.bvh";
    // 从bvh文件中读取所有的关节名称，需要用关节名称来找unity scene中的game object
    private List<string> jointNames = new List<string>();
    private List<string> jointNamesSecond = new List<string>();
    // game object列表
    private List<GameObject> gameObjects = new List<GameObject>();
    GameObject root;
    private int time_step;// 时间戳
    private int frame_num; // bvh的帧数
    private int frame_trans = 30;
    private int frame_num1_cut;
    private int frame_num1;
    private int frame_num2;
    
    // ! 在这里声明你需要的其他数据结构
    // 每个关节信息的记录
    public class Ajoint
    {
        public string name;
        public int id;//自己的id
        public int parent_id;// 父节点id
        public Vector3 offset;// 与父节点的距离
        public List<Vector3> AxisOrder = new List<Vector3>();//Rotation直接转化为轴向量
        //存储计算出的每一帧的旋转四元数，分开存
        public List<Quaternion> RotationPerFrame1 = new List<Quaternion>();
        public List<Quaternion> RotationPerFrame2 = new List<Quaternion>();
    }
    // 每一帧根节点初始位置，分开存
    public List<Vector3> m_PostionData1 = new List<Vector3>();
    public List<Vector3> m_PostionData2 = new List<Vector3>();
    // 第二个的id转化为第一个的id
    public List<int> func = new List<int>();
    public List<Ajoint> JOINTS = new List<Ajoint>(); // 记录关节信息
    private Ajoint tmpjoint = new Ajoint();
    private int joint_num_count;// 关节数目 -- 赋予新关节
    private Stack<int> st = new Stack<int>();// 存放当前关节结构的栈  - 存放关节的id

    List<Quaternion> last_rotation = new List<Quaternion>(); //转换前的朝向
    List<Quaternion> to_rotation = new List<Quaternion>(); //转换后的朝向
    List<List<Quaternion> > last_rotations = new List<List<Quaternion> >(); //转换前frame_trans帧的朝向
    List<List<Quaternion> > to_rotations = new List<List<Quaternion> >(); //转换后frame_trans帧的朝向
    Vector3 delta_y;//更改朝向
    Vector3 last_position;//转换前的位置
    Vector3 to_position;//转换前的位置
    List<Vector3> trans_positions = new List<Vector3>();
    int transition_time_step;
    //记录状态
    char state;//哪个动作片段
    bool pause;//是否暂停
    float frame_time;
        
    Quaternion rotationSlerp(Quaternion rota, Quaternion rotb, float t_f)
    {
        // 实现四元数的slerp插值
        Quaternion current_rot = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        float cosa = Quaternion.Dot(rota,rotb);
        float k0,k1;

        float aw = rota.w, ax = rota.x, ay = rota.y, az = rota.z;
        float bw = rotb.w, bx = rotb.x, by = rotb.y, bz = rotb.z;

        // 两个向量夹角为负
        if (cosa < 0.0f){
            bw = -bw;   bx = -bx;   by = -by;   bz = -bz;   cosa = -cosa;
        }
        if (cosa > 0.9995f){// 两个向量夹角几乎为0，直接线性插值
            k0 = 1.0f -t_f;
            k1 = t_f;
        }
        else{
            float sina = Mathf.Sqrt( 1.0f - cosa*cosa );
            float a = Mathf.Atan2( sina, cosa );
            k0 = Mathf.Sin((1.0f - t_f)*a)  / sina;
            k1 = Mathf.Sin(t_f*a) / sina;
        }
        
        current_rot.w = aw*k0 + bw*k1;  current_rot.x = ax*k0 + bx*k1;
        current_rot.y = ay*k0 + by*k1;  current_rot.z = az*k0 + bz*k1;
        return current_rot;
    }

    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = 70;
        time_step = 0;
        state = ' ';
        pause = false;
        transition_time_step = frame_trans;

        joint_num_count = 0;
        st.Push(0);
        StreamReader bvh_file1 = new StreamReader(new FileStream(bvh_fname1, FileMode.Open));
        while(!bvh_file1.EndOfStream){// 逐行读取
            string line = bvh_file1.ReadLine();
            string str = new System.Text.RegularExpressions.Regex("[\\s]+").Replace(line, " ");
            string[] split_line = str.Split(' ');
            
            // 处理bvh文件中的character hierarchy部分
            if (line.Contains("ROOT") || line.Contains("JOINT")){
                tmpjoint = new Ajoint();
                tmpjoint.name = split_line[split_line.Length-1];
                tmpjoint.id = joint_num_count;
                tmpjoint.parent_id = st.Peek();
                joint_num_count++;

                jointNames.Add(tmpjoint.name);
            } 
            else if (line.Contains("End Site")){
                tmpjoint = new Ajoint();
                tmpjoint.name = jointNames[st.Peek()] + "_end";
                tmpjoint.id = joint_num_count;
                tmpjoint.parent_id = st.Peek();
                joint_num_count++;
                     
                jointNames.Add(tmpjoint.name);
            }
            else if (line.Contains("{")){
                st.Push(tmpjoint.id);
            }
            else if (line.Contains("}")){
                st.Pop(); 
            }
            else if (line.Contains("OFFSET")){
                // 与父节点的距离
                int j = 0;
                for (; j < split_line.Length; j++)
                    if (split_line[j] == "OFFSET"){
                        j++;
                        break;
                    }
                float x = (float)System.Convert.ToDouble(split_line[j]);
                float y = (float)System.Convert.ToDouble(split_line[j+1]);
                float z = (float)System.Convert.ToDouble(split_line[j+2]);
                tmpjoint.offset = new Vector3(x,y,z);
                if(tmpjoint.name.Contains("_end"))
                    JOINTS.Add(tmpjoint);
            }
            else if (line.Contains("CHANNELS")){
                // 欧拉角的顺序
                int num = split_line.Length;
                for(int j = num - 3; j < num; j++){
                    if(split_line[j].Contains("Xrotation")) tmpjoint.AxisOrder.Add(Vector3.right);
                    else if(split_line[j].Contains("Yrotation")) tmpjoint.AxisOrder.Add(Vector3.up);
                    else if(split_line[j].Contains("Zrotation")) tmpjoint.AxisOrder.Add(Vector3.forward);
                }
                JOINTS.Add(tmpjoint);
                
            }
            else if (line.Contains("Frame Time")){
                 frame_time = float.Parse(split_line[split_line.Length - 1]);
                // Frame Time是数据部分前的最后一行，读到这一行后跳出循环
                break;
            }
            else if (line.Contains("Frames:")){
                // 获取帧数
                frame_num1 = int.Parse(split_line[split_line.Length - 1]);
            }
        }
        
        // 接下来处理bvh文件中的数据部分
        while(!bvh_file1.EndOfStream){
            string line = bvh_file1.ReadLine();//每一行是一帧的数据
            string str = new System.Text.RegularExpressions.Regex("[\\s]+").Replace(line, " ");
            string[] split_line = line.Split(new[] { ' ', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
            // 直接得到每一帧的(t0,R0,R1,R2,...) 利用 rotx roty rotz计算R
            // 保存每一帧根节点的offset到全局
            float px,py,pz;
            px = float.Parse(split_line[0]);
            py = float.Parse(split_line[1]);
            pz = float.Parse(split_line[2]);
            Vector3 pos = new Vector3(px,py,pz);
            m_PostionData1.Add(pos);

            // 保存每一帧中每一个节点的旋转角
            int i = 3;
            for (int index = 0; index < JOINTS.Count; index++){
                if(JOINTS[index].name.Contains("_end")) {
                    JOINTS[index].RotationPerFrame1.Add(Quaternion.identity);
                    continue;
                }
                float[] rot = new float[3];
                float.TryParse(split_line[i],out rot[0]);
                float.TryParse(split_line[i+1],out rot[1]);
                float.TryParse(split_line[i+2],out rot[2]);
                i = i + 3;
                Quaternion q = Quaternion.AngleAxis(rot[0],JOINTS[index].AxisOrder[0])
                               *Quaternion.AngleAxis(rot[1],JOINTS[index].AxisOrder[1])
                               *Quaternion.AngleAxis(rot[2],JOINTS[index].AxisOrder[2]);
                JOINTS[index].RotationPerFrame1.Add(q);  
            }
            
        }

        // 层级结构相同，但是顺序不同
        joint_num_count = 0;
        st.Push(0);
        StreamReader bvh_file2 = new StreamReader(new FileStream(bvh_fname2, FileMode.Open));
        while(!bvh_file2.EndOfStream){// 逐行读取
            string line = bvh_file2.ReadLine();
            string str = new System.Text.RegularExpressions.Regex("[\\s]+").Replace(line, " ");
            string[] split_line = str.Split(' ');
            
            // 处理bvh文件中的character hierarchy部分
            if (line.Contains("ROOT") || line.Contains("JOINT")){
                tmpjoint = new Ajoint();
                tmpjoint.name = split_line[split_line.Length-1];
                tmpjoint.id = joint_num_count;
                joint_num_count++;

                jointNamesSecond.Add(tmpjoint.name);
            } 
            else if (line.Contains("End Site")){
                tmpjoint = new Ajoint();
                tmpjoint.name = jointNamesSecond[st.Peek()] + "_end";
                tmpjoint.id = joint_num_count;
                joint_num_count++;
                     
                jointNamesSecond.Add(tmpjoint.name);
            }
            else if (line.Contains("{")){
                st.Push(tmpjoint.id);
            }
            else if (line.Contains("}")){
                st.Pop(); 
            }
            else if (line.Contains("OFFSET")){
                if(tmpjoint.name.Contains("_end")){                
                    int idx;
                    for ( idx = 0; idx < JOINTS.Count; ++idx)
                        if (tmpjoint.name == JOINTS[idx].name)
                            break;
                    func.Add(idx);
                }
            }
            else if (line.Contains("CHANNELS")){
                // 根据tmp_name找到对应的Ajoint
                int idx;
                for ( idx = 0; idx < JOINTS.Count; ++idx)
                    if (tmpjoint.name == JOINTS[idx].name){
                        break;
                    }
                func.Add(idx);
                // 欧拉角的顺序
                int num = split_line.Length;
                for(int j = num - 3; j < num; j++){
                    if(split_line[j].Contains("Xrotation")) JOINTS[idx].AxisOrder[j - (num - 3)] = (Vector3.right);
                    else if(split_line[j].Contains("Yrotation")) JOINTS[idx].AxisOrder[j - (num - 3)] = (Vector3.up);
                    else if(split_line[j].Contains("Zrotation")) JOINTS[idx].AxisOrder[j - (num - 3)] = (Vector3.forward);
                }
                
            }
            else if (line.Contains("Frame Time")){
                 frame_time = float.Parse(split_line[split_line.Length - 1]);
                // Frame Time是数据部分前的最后一行，读到这一行后跳出循环
                break;
            }
            else if (line.Contains("Frames:")){
                // 获取帧数
                frame_num2 = int.Parse(split_line[split_line.Length - 1]);
            }
        }
        
        // 接下来处理bvh文件中的数据部分
        while(!bvh_file2.EndOfStream){
            string line = bvh_file2.ReadLine();//每一行是一帧的数据
            string str = new System.Text.RegularExpressions.Regex("[\\s]+").Replace(line, " ");
            string[] split_line = line.Split(new[] { ' ', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
            // 直接得到每一帧的(t0,R0,R1,R2,...) 利用 rotx roty rotz计算R
            // 保存每一帧根节点的offset到全局
            float px,py,pz;
            px = float.Parse(split_line[0]);
            py = float.Parse(split_line[1]);
            pz = float.Parse(split_line[2]);
            Vector3 pos = new Vector3(px,py,pz);
            m_PostionData2.Add(pos);

            // 保存每一帧中每一个节点的旋转角
            int i = 3;
            for (int index = 0; index < JOINTS.Count; index++){
                if(jointNamesSecond[index].Contains("_end")) {
                    JOINTS[func[index]].RotationPerFrame2.Add(Quaternion.identity);
                    continue;
                }
                float[] rot = new float[3];
                float.TryParse(split_line[i],out rot[0]);
                float.TryParse(split_line[i+1],out rot[1]);
                float.TryParse(split_line[i+2],out rot[2]);
                i = i + 3;
                Quaternion q = Quaternion.AngleAxis(rot[0],JOINTS[func[index]].AxisOrder[0])
                               * Quaternion.AngleAxis(rot[1],JOINTS[func[index]].AxisOrder[1])
                               * Quaternion.AngleAxis(rot[2],JOINTS[func[index]].AxisOrder[2]);
                JOINTS[func[index]].RotationPerFrame2.Add(q);  
            }
        }
        
        
        // 按关节名称获取所有的game object
        GameObject tmp_obj = new GameObject();
        for (int i = 0; i < jointNames.Count; i++){
            tmp_obj = GameObject.Find(jointNames[i]);
            gameObjects.Add(tmp_obj);
            last_rotation.Add(Quaternion.identity);
            to_rotation.Add(Quaternion.identity);
        }
        // 截取
        frame_num1_cut = frame_num1 - 100;
        // 为按键1的插值做准备
        int t1 = frame_num1_cut - frame_trans,t2 = 0;          
        for(int t = 0; t < frame_trans; t ++){
            List<Quaternion> tmp1 = new List<Quaternion>();
            List<Quaternion> tmp2 = new List<Quaternion>();
            float t_f = t* 1.0f / frame_trans;
            trans_positions.Add( (1 - t_f) * m_PostionData1[t1+t] + t_f*(m_PostionData2[t2+t] - m_PostionData2[t2] + m_PostionData1[frame_num1_cut - 1] ));
            for(int i = 0; i < JOINTS.Count; ++i){
                if(i == 0){
                    tmp1.Add( JOINTS[i].RotationPerFrame1[t1+t]);
                    tmp2.Add( JOINTS[i].RotationPerFrame2[t2+t]);
                }
                else{
                    tmp1.Add( tmp1[JOINTS[i].parent_id] * JOINTS[i].RotationPerFrame1[t1+t]);
                    tmp2.Add( tmp2[JOINTS[i].parent_id] * JOINTS[i].RotationPerFrame2[t2+t]);
                }
            }
            last_rotations.Add(tmp1);
            to_rotations.Add(tmp2);
        }
        root = gameObjects[0];
        frame_num = frame_num1_cut + frame_num2 - frame_trans;

        Quaternion q1 = JOINTS[0].RotationPerFrame1[frame_num1-1];
        Quaternion q2 = JOINTS[0].RotationPerFrame2[0];
        delta_y.x = 0.0f;
        delta_y.z = 0.0f;
        delta_y.y = q1.eulerAngles.y - q2.eulerAngles.y;
        last_position = m_PostionData1[frame_num1 - 1];
    }

    // 记录最终position和rotation，便于插值，切换动作时with_transition == true，动画播放结束(一个bvh文件已运动完）/中间片段插值结束时with_transition == false
    void save_before_change(bool with_transition){
        last_position = gameObjects[0].transform.position;
        for (int i = 0; i < JOINTS.Count; ++i)
            last_rotation[i] = gameObjects[i].transform.rotation;
        time_step = 0;
        if(with_transition) transition_time_step = 0;
    }
    // 过渡片段，rotation朝向对齐，position先xz平面位置已经对齐，只需对position和rotation位姿插值，两点插值即可
    void transition(int time_step){
        float t_f = time_step * 1.0f / frame_trans;
        for (int i = 0; i < JOINTS.Count; ++i)
            gameObjects[i].transform.rotation = rotationSlerp(last_rotation[i],to_rotation[i],t_f);
        gameObjects[0].transform.position = last_position * (1 - t_f) + to_position * t_f;
    }
    // 利用从bvh中读入的信息，更新position和rotation
    // 控制朝向和位置的对齐：根节点rotation需要旋转delta_y，根节点position需要利用上次的位置last_position，加上每两帧间位移旋转delta_y
    void play_walk(bool for_transition){
        Vector3 joint_position = new Vector3(0.0F, 0.0F, 0.0F);
        Quaternion joint_orientation = Quaternion.identity;
        for (int i = 0; i < JOINTS.Count; i++){
            Ajoint cur_joint = JOINTS[i];
            if(i == 0){// root node
                joint_orientation = Quaternion.Euler(cur_joint.RotationPerFrame1[time_step].eulerAngles + delta_y);

                joint_position = new Vector3 (last_position.x, 0.0f, last_position.z)+  
                (Quaternion.Euler(delta_y) * new Vector3(m_PostionData1[time_step].x - m_PostionData1[0].x,0.0f,m_PostionData1[time_step].z - m_PostionData1[0].z));
                joint_position.y = m_PostionData1[time_step].y;
                
                if(for_transition){
                    to_rotation[i] = joint_orientation;
                    to_position = new Vector3 (last_position.x, 0.0f, last_position.z)+  
                    (Quaternion.Euler(delta_y) *(1.0f * new Vector3(m_PostionData1[1].x - m_PostionData1[0].x,0.0f,m_PostionData1[1].z - m_PostionData1[0].z)));
                    to_position.y = m_PostionData1[0].y;
                }
                else 
                    gameObjects[i].transform.position = joint_position;
            }
            else
                joint_orientation = gameObjects[cur_joint.parent_id].transform.rotation * cur_joint.RotationPerFrame1[time_step];
            // 更新每个关节的旋转
            if(for_transition)
                to_rotation[i] = joint_orientation;
            else
                gameObjects[i].transform.rotation = joint_orientation;
        }
    }

    void play_cartwheel(bool for_transition){
        Vector3 joint_position = new Vector3(0.0F, 0.0F, 0.0F);
        Quaternion joint_orientation = Quaternion.identity;
        for (int i = 0; i < JOINTS.Count; i++){
            Ajoint cur_joint = JOINTS[i];
            if(i == 0){// root node
                joint_orientation = Quaternion.Euler(cur_joint.RotationPerFrame2[time_step].eulerAngles + delta_y);
        
                joint_position = new Vector3 (last_position.x, 0.0f, last_position.z)+  
                (Quaternion.Euler(delta_y) * new Vector3(m_PostionData2[time_step].x - m_PostionData2[0].x,0.0f,m_PostionData2[time_step].z - m_PostionData2[0].z));
                joint_position.y = m_PostionData2[time_step].y;
                
                if(for_transition){
                    to_rotation[i] = joint_orientation;
                    to_position = new Vector3 (last_position.x, 0.0f, last_position.z)+  
                    (Quaternion.Euler(delta_y) *(1.0f * new Vector3(m_PostionData2[1].x - m_PostionData2[0].x,0.0f,m_PostionData2[1].z - m_PostionData2[0].z)));
                    to_position.y = m_PostionData2[0].y;
                    
                }
                else 
                    gameObjects[i].transform.position = joint_position;
            }
            else{
                joint_orientation = gameObjects[cur_joint.parent_id].transform.rotation * cur_joint.RotationPerFrame2[time_step];
            }
            // 更新每个关节的旋转  
            if(for_transition)
                to_rotation[i] = joint_orientation;
            else
                gameObjects[i].transform.rotation = joint_orientation;
        }
    }
    // 根据障碍物位置判断，避免撞上
    bool encouter_barrier(){
        // 避免撞上barrier 0.2
        bool return_value1 = ( Mathf.Abs(root.transform.position.z - 1.0f) < 0.5f && Mathf.Abs(root.transform.position.x - 1.5f) < 0.5f );
        if(return_value1){
            if(root.transform.position.z  < 1.0f){//从前面撞上
                root.transform.position = root.transform.position - new Vector3(0.0f,0.0f,0.08f);
                state = 'D';
                save_before_change(true);
                    delta_y.y = 90.0f;
            }
            if(root.transform.position.z  >= 1.0f ){// 从后面撞上
                root.transform.position = root.transform.position + new Vector3(0.0f,0.0f,0.08f);
                state = 'A';
                save_before_change(true);
                    delta_y.y = -90.0f;
            } 
        }
        // 避免撞上wall
        bool return_value2 = ( Mathf.Abs(root.transform.position.z - 5.0f) < 1.1f && Mathf.Abs(root.transform.position.x - (-5.0f) ) < 0.5f );
        if(return_value2){
            if( root.transform.position.x > -5.0f){//从右面撞上
                root.transform.position = root.transform.position + new Vector3(0.08f,0.0f,0.0f);
                state = 'W';
                save_before_change(true);
                    delta_y.y = 0.0f;
            }
            if(root.transform.position.x  <= -5.0f ){//从左面撞上
                root.transform.position = root.transform.position - new Vector3(0.08f,0.0f,0.0f);
                state = 'S';
                save_before_change(true);
                    delta_y.y = 180.0f;
            } 
        }
        return return_value1 || return_value2;
    }
    // 衔接walk和cartwheel
    void special_connection(){
        if (time_step < frame_num1_cut - frame_trans){//第一段
            for (int i = 0; i < JOINTS.Count; i++){
                Ajoint cur_joint = JOINTS[i];
                if(i == 0){// root node
                    gameObjects[i].transform.position = m_PostionData1[time_step];
                    gameObjects[i].transform.rotation = cur_joint.RotationPerFrame1[time_step];
                }
                else
                    gameObjects[i].transform.rotation = gameObjects[cur_joint.parent_id].transform.rotation * cur_joint.RotationPerFrame1[time_step];
            }
        }
        else if (time_step < frame_num1_cut){//过渡
            int time_step_stage = time_step - (frame_num1_cut - frame_trans);
            print(time_step_stage);
            float t_f = time_step_stage * 1.0f / frame_trans;
            for (int i = 0; i < JOINTS.Count; i++){
                if(i == 0){// root node
                    gameObjects[i].transform.position = trans_positions[time_step_stage];
                }
                // 更新每个关节的旋转
                gameObjects[i].transform.rotation = rotationSlerp(last_rotations[time_step_stage][i],to_rotations[time_step_stage][i],t_f);
            }
        }
        else if (time_step >= frame_num1_cut){//第二段
            if(time_step == frame_num1_cut) last_position = root.transform.position;
            int time_step_real = time_step - (frame_num1_cut - frame_trans);
            for (int i = 0; i < JOINTS.Count; i++){
                Vector3 joint_position;
                Ajoint cur_joint = JOINTS[i];
                if(i == 0){// root node
                    // 朝向没有对齐
                    gameObjects[i].transform.rotation = Quaternion.Euler(cur_joint.RotationPerFrame2[time_step_real].eulerAngles);
                    //(x,z)平面内的位置对齐，位移方向也对齐
                    joint_position = new Vector3 (last_position.x, 0.0f, last_position.z)+  
                    new Vector3(m_PostionData2[time_step_real].x - m_PostionData2[frame_trans - 1].x,0.0f,m_PostionData2[time_step_real].z - m_PostionData2[frame_trans - 1].z);
                    joint_position.y = m_PostionData2[time_step_real].y;
                    
                    gameObjects[i].transform.position = joint_position;
                }
                else
                    gameObjects[i].transform.rotation = gameObjects[cur_joint.parent_id].transform.rotation * cur_joint.RotationPerFrame2[time_step_real];
            }
        }
        // 更新时间戳
        time_step = (time_step + 1) % frame_num;
    }

    // Update is called once per frame
    void Update()
    {
        // 用 GetKey 或 GetKeyDown 或 GetKeyUp 交互
        // 先旋转到相应朝向，然后走
        if (Input.GetKeyDown(KeyCode.W))//前
        {
            state = 'W';
            save_before_change(true);
            delta_y.y = 0.0f;
        }
        if (Input.GetKeyDown(KeyCode.A))//左
        {
            state = 'A';
            save_before_change(true);
            delta_y.y = -90.0f;
        }
        if (Input.GetKeyDown(KeyCode.S))//后
        {
            state = 'S';
            save_before_change(true);
            delta_y.y = 180.0f;
        }
        if (Input.GetKeyDown(KeyCode.D))//右
        {
            state = 'D';
            save_before_change(true);
            delta_y.y = 90.0f;
        }

        if (Input.GetKey(KeyCode.Q))//turn left
        {
            state = 'Z';
            root.transform.rotation = Quaternion.Euler(new Vector3(0.0f,-1.0f,0.0f)) * root.transform.rotation;
        }
        if (Input.GetKey(KeyCode.E))//turn right
        {
            state = 'Z';
            root.transform.rotation = Quaternion.Euler(new Vector3(0.0f,1.0f,0.0f)) * root.transform.rotation;
        }
        if (Input.GetKeyDown(KeyCode.Z))//静止
        {
            if (!pause) {pause = true; print("pause");print(state);print(time_step);print(root.transform.position);print(last_position);}
            else {pause = false; print("start moving");print(state);print(time_step);print(root.transform.position);print(last_position);}

        }
        if (Input.GetKeyDown(KeyCode.Alpha1)) // 专门连接walk与cartwheel
        {
            state = '1';
            time_step = 0;
            //save_before_change(false);
        }
        if (Input.GetKeyDown(KeyCode.Alpha2)) // 接着上一次的位置和朝向的动作walk，所以预先计算好朝向要旋转的角度
        {
            state = '2';  
            save_before_change(true);
            delta_y.y = last_rotation[0].eulerAngles.y - JOINTS[0].RotationPerFrame1[0].eulerAngles.y;
        }
        if (Input.GetKeyDown(KeyCode.Alpha3)) // 接着上一次的位置和朝向的动作cartwheel，所以预先计算好朝向要旋转的角度
        {
            state = '3';
            save_before_change(true);
            delta_y.y = last_rotation[0].eulerAngles.y -  JOINTS[0].RotationPerFrame2[0].eulerAngles.y;
        }
        
        if(pause || state == 'Z'){
            //print("pause this frame");
        }
        else if(transition_time_step < frame_trans){
            // 开始：获取插值信息
            if(transition_time_step == 0){
                print("start transition");
                print(root.transform.position);
                if(state == '1' || state == '2' || state == 'W' || state == 'S' || state == 'A' || state == 'D') play_walk(true);
                else if(state == '3') play_cartwheel(true);
            }
            //插值
            transition(transition_time_step);
            // 结束：更新位置和朝向
            if(transition_time_step == frame_trans - 1) {
                save_before_change(false);
                print(root.transform.position);
                print("end transition");
            }
            transition_time_step++;
        }
        // 有xz平面的位置对齐和绕y轴旋转角度(朝向)对齐，和x z方向的rotation的插值
        else if (state == '2' || state == 'W' || state == 'S' || state == 'A' || state == 'D'){
            if(time_step == 0){
                print("start walk");
                print(gameObjects[0].transform.position);
            }
            play_walk(false);
            time_step = (time_step + 1) % frame_num1;
            if(time_step == 0)
                save_before_change(false);
        }
        else if (state == '3'){
            if(time_step == 0){
                print("start cartwheel");
                print(gameObjects[0].transform.position);
            }
            play_cartwheel(false);
            time_step = (time_step + 1) % frame_num2;
            if(time_step == 0)
                save_before_change(false);
        }
        else if (state == '1')
            special_connection();
        // 自动避开障碍物
        encouter_barrier();
    }
}
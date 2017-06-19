//Q1
while (n>0){
	int d = n%2;
	stack.push(d);
	n=n/2;
	while (stack.isEmpty != true){
		System.out.print(stack.pop())
	}

}




//Q2
isEqual(s1,s2)
stack S = new stack()
a = s1.pop();
b = s2.pop();

if (a == b){
	s.push(a);
	if (s1.notEmpty || s2.notEmpty && a!=b){
		return false}
	else{
		while s.isEmpty != true{
			c = s.pop();
			s1.push(c);
			s2.push(c);
		} 
	}	
	}




//Q3
Public void Add5(Node root){
	if (root != null){
		Add5(root.left);
		root.Data = root.Data+5;
		Add5(root.right)
	}
}





//Q5 
Public int Rtmost(Node root){
	while (root.right != null && root.left != null){
		if (root.right == null){
			root=root.left;
		}else{
			root = root.right;
		}
	}System.out.print(root.data());
}



//Q6
Public void new(Node root){
	if (root != null){
		if (root.data > max){
			root = root.data;
			new(root.left);
			new(root.right);
		}
	}
}
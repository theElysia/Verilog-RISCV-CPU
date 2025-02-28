int fibb(int a);

int main(){
    int ans,n;
    n=6;
    ans=fibb(n);
    return 0;
}

int fibb(int a){
    if(a<=1)return 1;
    return fibb(a-1)+fibb(a-2);
}